#![macro_use]

use core::future::Future;
use core::pin::Pin;
use core::sync::atomic::{AtomicU32, AtomicUsize, Ordering, compiler_fence, fence};
use core::task::{Context, Poll};

use embassy_sync::waitqueue::AtomicWaker;
use linked_list::Table;

use super::word::{Word, WordSize};
use super::{Channel, Dir, Request, STATE};
use crate::interrupt::typelevel::Interrupt;
use crate::pac;
use crate::pac::gpdma::vals;
use crate::rcc::WakeGuard;

pub mod linked_list;
pub mod ringbuffered;

/// Diagnostic (bench) counters of `request_reset`, per channel id:
/// resets requested, and resets after which CxCR.EN still read 1.
static DIAG_RESETS: [AtomicU32; super::CHANNEL_COUNT] = [const { AtomicU32::new(0) }; super::CHANNEL_COUNT];
static DIAG_EN_AFTER_RESET: [AtomicU32; super::CHANNEL_COUNT] =
    [const { AtomicU32::new(0) }; super::CHANNEL_COUNT];
/// CxCR / CxSR read right after the last RESET write, per channel id.
static DIAG_LAST_RESET_CR: [AtomicU32; super::CHANNEL_COUNT] =
    [const { AtomicU32::new(0) }; super::CHANNEL_COUNT];
static DIAG_LAST_RESET_SR: [AtomicU32; super::CHANNEL_COUNT] =
    [const { AtomicU32::new(0) }; super::CHANNEL_COUNT];
/// USART UE cycles of the receiver recovery, by its RX channel id.
static DIAG_UE_CYCLES: [AtomicU32; super::CHANNEL_COUNT] =
    [const { AtomicU32::new(0) }; super::CHANNEL_COUNT];

/// Diagnostic: one UE cycle of the USART whose RX uses channel `id`.
pub(crate) fn diag_note_ue_cycle(id: u8) {
    let id = usize::from(id).min(super::CHANNEL_COUNT - 1);
    DIAG_UE_CYCLES[id].fetch_add(1, Ordering::Relaxed);
}

/// Diagnostic `request_reset` record of one channel.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct DiagResetCounts {
    /// Resets requested.
    pub resets: u32,
    /// Resets after which CxCR.EN still read 1.
    pub en_after_reset: u32,
    /// CxCR read right after the last RESET write.
    pub last_cr: u32,
    /// CxSR read right after the last RESET write.
    pub last_sr: u32,
    /// USART UE cycles of the receiver recovery on this RX channel.
    pub ue_cycles: u32,
}

/// Diagnostic (bench): the `request_reset` record of channel `id` (the
/// global DMA channel index).
pub fn diag_reset_counts(id: usize) -> DiagResetCounts {
    let id = id.min(super::CHANNEL_COUNT - 1);
    DiagResetCounts {
        resets: DIAG_RESETS[id].load(Ordering::Relaxed),
        en_after_reset: DIAG_EN_AFTER_RESET[id].load(Ordering::Relaxed),
        last_cr: DIAG_LAST_RESET_CR[id].load(Ordering::Relaxed),
        last_sr: DIAG_LAST_RESET_SR[id].load(Ordering::Relaxed),
        ue_cycles: DIAG_UE_CYCLES[id].load(Ordering::Relaxed),
    }
}

pub(crate) struct ChannelInfo {
    pub(crate) dma: pac::gpdma::Gpdma,
    pub(crate) num: usize,
    #[cfg(feature = "_dual-core")]
    pub(crate) irq: pac::Interrupt,
    #[cfg(feature = "low-power")]
    pub(crate) stop_mode: crate::rcc::StopMode,
}

impl ChannelInfo {
    fn wake_guard(&self) -> WakeGuard {
        WakeGuard::new(
            #[cfg(feature = "low-power")]
            self.stop_mode,
        )
    }
}

/// DMA request priority
#[derive(Debug, Copy, Clone, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Priority {
    /// Low Priority
    Low,
    /// Medium Priority
    Medium,
    /// High Priority
    High,
    /// Very High Priority
    VeryHigh,
}

impl From<Priority> for pac::gpdma::vals::Prio {
    fn from(value: Priority) -> Self {
        match value {
            Priority::Low => pac::gpdma::vals::Prio::LOW_WITH_LOWH_WEIGHT,
            Priority::Medium => pac::gpdma::vals::Prio::LOW_WITH_MID_WEIGHT,
            Priority::High => pac::gpdma::vals::Prio::LOW_WITH_HIGH_WEIGHT,
            Priority::VeryHigh => pac::gpdma::vals::Prio::HIGH,
        }
    }
}

/// GPDMA transfer options.
#[derive(Debug, Copy, Clone, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
pub struct TransferOptions {
    /// Request priority level.
    pub priority: Priority,
    /// Enable half transfer interrupt.
    pub half_transfer_ir: bool,
    /// Enable transfer complete interrupt.
    pub complete_transfer_ir: bool,
}

impl Default for TransferOptions {
    fn default() -> Self {
        Self {
            priority: Priority::VeryHigh,
            half_transfer_ir: false,
            complete_transfer_ir: true,
        }
    }
}

impl From<WordSize> for vals::Dw {
    fn from(raw: WordSize) -> Self {
        match raw {
            WordSize::OneByte => Self::BYTE,
            WordSize::TwoBytes => Self::HALF_WORD,
            WordSize::FourBytes => Self::WORD,
            _ => panic!("Invalid word size"),
        }
    }
}

impl From<vals::Dw> for WordSize {
    fn from(raw: vals::Dw) -> Self {
        match raw {
            vals::Dw::BYTE => Self::OneByte,
            vals::Dw::HALF_WORD => Self::TwoBytes,
            vals::Dw::WORD => Self::FourBytes,
            _ => panic!("Invalid word size"),
        }
    }
}

pub(crate) struct LLiState {
    /// The number of linked-list items.
    count: AtomicUsize,
    /// The index of the current linked-list item.
    index: AtomicUsize,
    /// The total transfer count of all linked-list items in number of words.
    transfer_count: AtomicUsize,
}

pub(crate) struct ChannelState {
    waker: AtomicWaker,
    complete_count: AtomicUsize,
    lli_state: LLiState,
}

impl ChannelState {
    pub(crate) const NEW: Self = Self {
        waker: AtomicWaker::new(),
        complete_count: AtomicUsize::new(0),

        lli_state: LLiState {
            count: AtomicUsize::new(0),
            index: AtomicUsize::new(0),
            transfer_count: AtomicUsize::new(0),
        },
    };
}

/// safety: must be called only once
pub(crate) unsafe fn init(cs: critical_section::CriticalSection, irq_priority: crate::interrupt::Priority) {
    foreach_interrupt! {
        ($peri:ident, gpdma, $block:ident, $signal_name:ident, $irq:ident) => {
            crate::interrupt::typelevel::$irq::set_priority_with_cs(cs, irq_priority);
            #[cfg(not(feature = "_dual-core"))]
            crate::interrupt::typelevel::$irq::enable();
        };
    }
    crate::_generated::init_gpdma();
}

pub(crate) unsafe fn on_irq(id: u8) {
    let info = super::info(id);
    #[cfg(feature = "_dual-core")]
    {
        use embassy_hal_internal::interrupt::InterruptExt as _;
        info.irq.enable();
    }

    let state = &STATE[id as usize];

    let ch = info.dma.ch(info.num);
    let sr = ch.sr().read();

    if sr.dtef() {
        panic!(
            "DMA: data transfer error on DMA@{:08x} channel {}",
            info.dma.as_ptr() as u32,
            info.num
        );
    }
    if sr.usef() {
        panic!(
            "DMA: user settings error on DMA@{:08x} channel {}",
            info.dma.as_ptr() as u32,
            info.num
        );
    }
    if sr.ulef() {
        panic!(
            "DMA: link transfer error on DMA@{:08x} channel {}",
            info.dma.as_ptr() as u32,
            info.num
        );
    }

    if sr.htf() {
        ch.fcr().write(|w| w.set_htf(true));
    }

    if sr.tcf() {
        ch.fcr().write(|w| w.set_tcf(true));

        let lli_count = state.lli_state.count.load(Ordering::Acquire);
        let complete = if lli_count > 0 {
            let next_lli_index = state.lli_state.index.load(Ordering::Acquire) + 1;
            let complete = next_lli_index >= lli_count;

            state
                .lli_state
                .index
                .store(if complete { 0 } else { next_lli_index }, Ordering::Release);

            complete
        } else {
            true
        };

        if complete {
            state.complete_count.fetch_add(1, Ordering::Release);
        }
    }

    if sr.suspf() {
        // Disable all xxIEs to prevent the irq from firing again. Only the
        // enables: writing the whole register would also clear SUSP, i.e.
        // resume the channel the caller just suspended.
        ch.cr().modify(|w| {
            w.set_tcie(false);
            w.set_htie(false);
            w.set_dteie(false);
            w.set_uleie(false);
            w.set_useie(false);
            w.set_suspie(false);
            w.set_toie(false);
        });
    }
    state.waker.wake();
}

impl<'d> Channel<'d> {
    fn info(&self) -> &'static super::ChannelInfo {
        super::info(self.id)
    }

    fn get_remaining_transfers(&self) -> u16 {
        let info = self.info();
        let ch = info.dma.ch(info.num);
        let word_size: WordSize = ch.tr1().read().ddw().into();

        ch.br1().read().bndt() / word_size.bytes() as u16
    }

    /// Raw channel registers for diagnostics: `(SR, CR, BR1, LLI index)`.
    /// Register reads only, no side effect.
    pub(crate) fn diag_regs(&self) -> (u32, u32, u32, u32) {
        let info = self.info();
        let ch = info.dma.ch(info.num);
        let lli = STATE[self.id as usize].lli_state.index.load(Ordering::Relaxed) as u32;
        (ch.sr().read().0, ch.cr().read().0, ch.br1().read().0, lli)
    }

    /// Diagnostic: `(GPDMA block address, channel number in the block, global
    /// channel id)`.
    pub(crate) fn diag_ids(&self) -> (usize, u8, u8) {
        let info = self.info();
        (info.dma.as_ptr() as usize, info.num as u8, self.id)
    }

    /// Whether a transfer-complete is flagged that the IRQ handler has not
    /// serviced yet (its LLI bookkeeping is then one item behind the hardware).
    pub(crate) fn tc_pending(&self) -> bool {
        let info = self.info();
        info.dma.ch(info.num).sr().read().tcf()
    }

    unsafe fn configure(
        &self,
        request: Request,
        dir: Dir,
        peri_addr: *const u32,
        mem_addr: *mut u32,
        mem_len: usize,
        incr_mem: bool,
        data_size: WordSize,
        dst_size: WordSize,
        options: TransferOptions,
    ) {
        // BNDT is specified as bytes, not as number of transfers.
        let Ok(bndt) = (mem_len * data_size.bytes()).try_into() else {
            panic!("DMA transfers may not be larger than 65535 bytes.");
        };

        let info = self.info();
        let ch = info.dma.ch(info.num);

        // "Preceding reads and writes cannot be moved past subsequent writes."
        fence(Ordering::SeqCst);

        if ch.cr().read().en() {
            ch.cr().modify(|w| w.set_susp(true));
            while !ch.sr().read().suspf() {}
        }

        ch.cr().write(|w| w.set_reset(true));
        ch.fcr().write(|w| {
            // Clear all irqs
            w.set_dtef(true);
            w.set_htf(true);
            w.set_suspf(true);
            w.set_tcf(true);
            w.set_tof(true);
            w.set_ulef(true);
            w.set_usef(true);
        });
        ch.llr().write(|_| {}); // no linked list
        ch.tr1().write(|w| {
            w.set_sdw(data_size.into());
            w.set_ddw(dst_size.into());
            w.set_sinc(dir == Dir::MemoryToPeripheral && incr_mem);
            w.set_dinc(dir == Dir::PeripheralToMemory && incr_mem);
            w.set_dap(match dir {
                Dir::MemoryToPeripheral => vals::Ap::PORT1, // Destination is peripheral on AHB for HPDMA
                Dir::PeripheralToMemory => vals::Ap::PORT0, // Destination is memory on AXI for HPDMA
                Dir::MemoryToMemory => panic!("memory-to-memory transfers not implemented for GPDMA"),
            });
            w.set_sap(match dir {
                Dir::MemoryToPeripheral => vals::Ap::PORT0, // Source is memory on AXI for HPDMA
                Dir::PeripheralToMemory => vals::Ap::PORT1, // Source is peripheral on AHB for HPDMA
                Dir::MemoryToMemory => panic!("memory-to-memory transfers not implemented for GPDMA"),
            });
        });
        ch.tr2().write(|w| {
            w.set_dreq(match dir {
                Dir::MemoryToPeripheral => vals::Dreq::DESTINATION_PERIPHERAL,
                Dir::PeripheralToMemory => vals::Dreq::SOURCE_PERIPHERAL,
                Dir::MemoryToMemory => panic!("memory-to-memory transfers not implemented for GPDMA"),
            });
            w.set_reqsel(request);
        });
        ch.tr3().write(|_| {}); // no address offsets.
        ch.br1().write(|w| w.set_bndt(bndt));

        match dir {
            Dir::MemoryToPeripheral => {
                ch.sar().write_value(mem_addr as _);
                ch.dar().write_value(peri_addr as _);
            }
            Dir::PeripheralToMemory => {
                ch.sar().write_value(peri_addr as _);
                ch.dar().write_value(mem_addr as _);
            }
            Dir::MemoryToMemory => panic!("memory-to-memory transfers not implemented for GPDMA"),
        }

        ch.cr().write(|w| {
            w.set_prio(options.priority.into());
            w.set_htie(options.half_transfer_ir);
            w.set_tcie(options.complete_transfer_ir);
            w.set_useie(true);
            w.set_dteie(true);
            w.set_suspie(true);
        });

        let state = &STATE[self.id as usize];
        state.lli_state.count.store(0, Ordering::Relaxed);
        state.lli_state.index.store(0, Ordering::Relaxed);
        state.lli_state.transfer_count.store(0, Ordering::Relaxed)
    }

    /// Configure a linked-list transfer.
    unsafe fn configure_linked_list<const ITEM_COUNT: usize>(
        &self,
        table: &Table<ITEM_COUNT>,
        options: TransferOptions,
    ) {
        let info = self.info();
        let ch = info.dma.ch(info.num);

        // "Preceding reads and writes cannot be moved past subsequent writes."
        fence(Ordering::SeqCst);

        ch.cr().write(|w| w.set_reset(true));
        ch.fcr().write(|w| {
            // Clear all irqs
            w.set_dtef(true);
            w.set_htf(true);
            w.set_suspf(true);
            w.set_tcf(true);
            w.set_tof(true);
            w.set_ulef(true);
            w.set_usef(true);
        });
        ch.lbar().write(|reg| reg.set_lba(table.base_address()));

        // Empty LLI0.
        ch.br1().write(|w| w.set_bndt(0));

        // Enable all linked-list field updates.
        ch.llr().write(|w| {
            w.set_ut1(true);
            w.set_ut2(true);
            w.set_ub1(true);
            w.set_usa(true);
            w.set_uda(true);
            w.set_ull(true);

            // Lower two bits are ignored: 32 bit aligned.
            w.set_la(table.offset_address(0) >> 2);
        });

        ch.tr3().write(|_| {}); // no address offsets.

        ch.cr().write(|w| {
            w.set_prio(options.priority.into());
            w.set_htie(options.half_transfer_ir);
            w.set_tcie(options.complete_transfer_ir);
            w.set_useie(true);
            w.set_uleie(true);
            w.set_dteie(true);
            w.set_suspie(true);
        });

        let state = &STATE[self.id as usize];
        state.lli_state.count.store(ITEM_COUNT, Ordering::Relaxed);
        state.lli_state.index.store(0, Ordering::Relaxed);
        state
            .lli_state
            .transfer_count
            .store(table.transfer_count(), Ordering::Relaxed)
    }

    fn start(&self) {
        let info = self.info();
        let ch = info.dma.ch(info.num);

        ch.cr().modify(|w| w.set_en(true));
    }

    fn request_pause(&self) {
        let info = self.info();
        let ch = info.dma.ch(info.num);

        ch.cr().modify(|w| w.set_susp(true))
    }

    fn request_resume(&self) {
        let info = self.info();
        let ch = info.dma.ch(info.num);

        ch.cr().modify(|w| w.set_susp(false));
    }

    fn request_reset(&self) {
        let info = self.info();
        let ch = info.dma.ch(info.num);

        // CR.RESET only takes effect on a channel in a steady state:
        // suspended (EN = 1 with SUSPF = 1) or disabled (EN = 0). IDLEF alone
        // is not enough, and SUSPF must not be cleared first: on a channel
        // already suspended no new SUSPF follows, the RESET write is then
        // ignored and the channel stays enabled, stuck (embedded issue
        // 1695). A suspend whose SUSPF was cleared earlier is redone (resume,
        // suspend) to raise a fresh one. The RESET is checked and retried
        // once.
        fn suspended(ch: pac::gpdma::Channel) -> bool {
            for _ in 0..10_000 {
                if ch.sr().read().suspf() || !ch.cr().read().en() {
                    return true;
                }
            }
            false
        }
        let id = self.id as usize;
        for _ in 0..2 {
            if ch.cr().read().en() {
                ch.cr().modify(|w| w.set_susp(true));
                if !suspended(ch) {
                    ch.cr().modify(|w| w.set_susp(false));
                    ch.cr().modify(|w| w.set_susp(true));
                    suspended(ch);
                }
            }

            ch.cr().modify(|w| w.set_reset(true));

            // Diagnostic only: register reads and counters.
            let cr = ch.cr().read();
            DIAG_RESETS[id].fetch_add(1, Ordering::Relaxed);
            if cr.en() {
                DIAG_EN_AFTER_RESET[id].fetch_add(1, Ordering::Relaxed);
            }
            DIAG_LAST_RESET_CR[id].store(cr.0, Ordering::Relaxed);
            DIAG_LAST_RESET_SR[id].store(ch.sr().read().0, Ordering::Relaxed);
            if !cr.en() {
                break;
            }
        }
    }

    fn is_running(&self) -> bool {
        let info = self.info();
        let ch = info.dma.ch(info.num);

        let sr = ch.sr().read();

        !sr.suspf() && !sr.idlef()
    }

    fn poll_stop(&self) -> Poll<()> {
        compiler_fence(Ordering::SeqCst);

        if !self.is_running() {
            fence(Ordering::Acquire);

            Poll::Ready(())
        } else {
            Poll::Pending
        }
    }

    /// Create a read DMA transfer (peripheral to memory).
    pub unsafe fn read<'a, W: Word>(
        &'a mut self,
        request: Request,
        peri_addr: *mut W,
        buf: &'a mut [W],
        options: TransferOptions,
    ) -> Transfer<'a> {
        self.read_raw(request, peri_addr, buf, options)
    }

    /// Create a read DMA transfer (peripheral to memory), using raw pointers.
    pub unsafe fn read_raw<'a, MW: Word, PW: Word>(
        &'a mut self,
        request: Request,
        peri_addr: *mut PW,
        buf: *mut [MW],
        options: TransferOptions,
    ) -> Transfer<'a> {
        let mem_len = buf.len();
        assert!(mem_len > 0 && mem_len <= 0xFFFF);

        self.configure(
            request,
            Dir::PeripheralToMemory,
            peri_addr as *const u32,
            buf as *mut MW as *mut u32,
            mem_len,
            true,
            PW::size(),
            MW::size(),
            options,
        );
        self.start();

        Transfer {
            _wake_guard: self.info().wake_guard(),
            channel: self.reborrow(),
        }
    }

    /// Create a write DMA transfer (memory to peripheral).
    pub unsafe fn write<'a, MW: Word, PW: Word>(
        &'a mut self,
        request: Request,
        buf: &'a [MW],
        peri_addr: *mut PW,
        options: TransferOptions,
    ) -> Transfer<'a> {
        self.write_raw(request, buf, peri_addr, options)
    }

    /// Create a write DMA transfer (memory to peripheral), using raw pointers.
    pub unsafe fn write_raw<'a, MW: Word, PW: Word>(
        &'a mut self,
        request: Request,
        buf: *const [MW],
        peri_addr: *mut PW,
        options: TransferOptions,
    ) -> Transfer<'a> {
        let mem_len = buf.len();
        assert!(mem_len > 0 && mem_len <= 0xFFFF);

        self.configure(
            request,
            Dir::MemoryToPeripheral,
            peri_addr as *const u32,
            buf as *const MW as *mut u32,
            mem_len,
            true,
            MW::size(),
            PW::size(),
            options,
        );
        self.start();

        Transfer {
            _wake_guard: self.info().wake_guard(),
            channel: self.reborrow(),
        }
    }

    /// Create a write DMA transfer (memory to peripheral), writing the same value repeatedly.
    pub unsafe fn write_repeated<'a, MW: Word, PW: Word>(
        &'a mut self,
        request: Request,
        repeated: &'a MW,
        count: usize,
        peri_addr: *mut PW,
        options: TransferOptions,
    ) -> Transfer<'a> {
        assert!(count > 0 && count <= 0xFFFF);

        self.configure(
            request,
            Dir::MemoryToPeripheral,
            peri_addr as *const u32,
            repeated as *const MW as *mut u32,
            count,
            false,
            MW::size(),
            PW::size(),
            options,
        );
        self.start();

        Transfer {
            _wake_guard: self.info().wake_guard(),
            channel: self.reborrow(),
        }
    }

    /// Create a linked-list DMA transfer.
    pub unsafe fn linked_list<'a, const ITEM_COUNT: usize>(
        &'a mut self,
        table: Table<ITEM_COUNT>,
        options: TransferOptions,
    ) -> LinkedListTransfer<'a, ITEM_COUNT> {
        self.configure_linked_list(&table, options);
        self.start();

        LinkedListTransfer {
            _wake_guard: self.info().wake_guard(),
            channel: self.reborrow(),
        }
    }
}

/// Linked-list DMA transfer.
#[must_use = "futures do nothing unless you `.await` or poll them"]
pub struct LinkedListTransfer<'a, const ITEM_COUNT: usize> {
    channel: Channel<'a>,
    _wake_guard: WakeGuard,
}

impl<'a, const ITEM_COUNT: usize> LinkedListTransfer<'a, ITEM_COUNT> {
    /// Request the transfer to pause, keeping the existing configuration for this channel.
    ///
    /// To resume the transfer, call [`request_resume`](Self::request_resume) again.
    /// This doesn't immediately stop the transfer, you have to wait until [`is_running`](Self::is_running) returns false.
    pub fn request_pause(&mut self) {
        self.channel.request_pause()
    }

    /// Request the transfer to resume after having been paused.
    pub fn request_resume(&mut self) {
        self.channel.request_resume()
    }

    /// Request the DMA to reset.
    ///
    /// The configuration for this channel will **not be preserved**. If you need to restart the transfer
    /// at a later point with the same configuration, see [`request_pause`](Self::request_pause) instead.
    pub fn request_reset(&mut self) {
        self.channel.request_reset()
    }

    /// Return whether this transfer is still running.
    ///
    /// If this returns `false`, it can be because either the transfer finished, or
    /// it was requested to stop early with [`request_pause`](Self::request_pause).
    pub fn is_running(&mut self) -> bool {
        self.channel.is_running()
    }

    /// Gets the total remaining transfers for the channel
    /// Note: this will be zero for transfers that completed without cancellation.
    pub fn get_remaining_transfers(&self) -> u16 {
        self.channel.get_remaining_transfers()
    }

    /// Blocking wait until the transfer finishes.
    pub fn blocking_wait(mut self) {
        while self.is_running() {}

        // "Subsequent reads and writes cannot be moved ahead of preceding reads."
        fence(Ordering::SeqCst);

        core::mem::forget(self);
    }
}

impl<'a, const ITEM_COUNT: usize> Drop for LinkedListTransfer<'a, ITEM_COUNT> {
    fn drop(&mut self) {
        self.request_reset();

        // "Subsequent reads and writes cannot be moved ahead of preceding reads."
        fence(Ordering::SeqCst);
    }
}

impl<'a, const ITEM_COUNT: usize> Unpin for LinkedListTransfer<'a, ITEM_COUNT> {}
impl<'a, const ITEM_COUNT: usize> Future for LinkedListTransfer<'a, ITEM_COUNT> {
    type Output = ();
    fn poll(mut self: Pin<&mut Self>, cx: &mut Context<'_>) -> Poll<Self::Output> {
        let state = &STATE[self.channel.id as usize];
        state.waker.register(cx.waker());

        if self.is_running() {
            Poll::Pending
        } else {
            fence(Ordering::Acquire);

            Poll::Ready(())
        }
    }
}

/// DMA transfer.
#[must_use = "futures do nothing unless you `.await` or poll them"]
pub struct Transfer<'a> {
    channel: Channel<'a>,
    _wake_guard: WakeGuard,
}

impl<'a> Transfer<'a> {
    /// Request the transfer to pause, keeping the existing configuration for this channel.
    /// To restart the transfer, call [`start`](Self::start) again.
    ///
    /// This doesn't immediately stop the transfer, you have to wait until [`is_running`](Self::is_running) returns false.
    pub fn request_pause(&mut self) {
        self.channel.request_pause()
    }

    /// Request the transfer to resume after being suspended.
    pub fn request_resume(&mut self) {
        self.channel.request_resume()
    }

    /// Request the DMA to reset.
    ///
    /// The configuration for this channel will **not be preserved**. If you need to restart the transfer
    /// at a later point with the same configuration, see [`request_pause`](Self::request_pause) instead.
    pub fn request_reset(&mut self) {
        self.channel.request_reset()
    }

    /// Return whether this transfer is still running.
    ///
    /// If this returns `false`, it can be because either the transfer finished, or
    /// it was requested to stop early with [`request_pause`](Self::request_pause).
    pub fn is_running(&mut self) -> bool {
        self.channel.is_running()
    }

    /// Gets the total remaining transfers for the channel
    /// Note: this will be zero for transfers that completed without cancellation.
    pub fn get_remaining_transfers(&self) -> u16 {
        self.channel.get_remaining_transfers()
    }

    /// Blocking wait until the transfer finishes.
    pub fn blocking_wait(mut self) {
        while self.is_running() {}

        // "Subsequent reads and writes cannot be moved ahead of preceding reads."
        fence(Ordering::SeqCst);

        core::mem::forget(self);
    }

    pub(crate) unsafe fn unchecked_extend_lifetime(self) -> Transfer<'static> {
        unsafe { core::mem::transmute(self) }
    }
}

impl<'a> Drop for Transfer<'a> {
    fn drop(&mut self) {
        self.request_pause();
        while self.is_running() {}

        // "Subsequent reads and writes cannot be moved ahead of preceding reads."
        fence(Ordering::SeqCst);
    }
}

impl<'a> Unpin for Transfer<'a> {}
impl<'a> Future for Transfer<'a> {
    type Output = ();
    fn poll(mut self: Pin<&mut Self>, cx: &mut Context<'_>) -> Poll<Self::Output> {
        let state = &STATE[self.channel.id as usize];
        state.waker.register(cx.waker());

        compiler_fence(Ordering::SeqCst);
        if self.is_running() {
            Poll::Pending
        } else {
            fence(Ordering::Acquire);

            Poll::Ready(())
        }
    }
}
