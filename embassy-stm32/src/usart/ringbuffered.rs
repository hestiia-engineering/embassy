use core::future::poll_fn;
use core::mem;
use core::sync::atomic::{Ordering, compiler_fence};
use core::task::Poll;

use embassy_embedded_hal::SetConfig;
use embedded_io_async::ReadReady;
use futures_util::future::{Either, select};

use super::{
    Config, ConfigError, Error, Info, State, UartRx, clear_interrupt_flags, rdr, reconfigure, set_baudrate, sr,
};
use crate::dma::ReadableRingBuffer;
use crate::gpio::Flex;
use crate::mode::Async;
use crate::time::Hertz;
use crate::usart::Regs;

/// Rx-only Ring-buffered UART Driver
///
/// Created with [UartRx::into_ring_buffered]
///
/// ### Notes on 'waiting for bytes'
///
/// The `read(buf)` (but not `read()`) and `read_exact(buf)` functions
/// may need to wait for bytes to arrive, if the ring buffer does not
/// contain enough bytes to fill the buffer passed by the caller of
/// the function, or is empty.
///
/// Waiting for bytes operates in one of three modes, depending on
/// the behavior of the sender, the size of the buffer passed
/// to the function, and the configuration:
///
/// - If the sender sends intermittently, the 'idle line'
/// condition will be detected when the sender stops, and any
/// bytes in the ring buffer will be returned. If there are no
/// bytes in the buffer, the check will be repeated each time the
/// 'idle line' condition is detected, so if the sender sends just
/// a single byte, it will be returned once the 'idle line'
/// condition is detected.
///
/// - If the sender sends continuously, the call will wait until
/// the DMA controller indicates that it has written to either the
/// middle byte or last byte of the ring buffer ('half transfer'
/// or 'transfer complete', respectively). This does not indicate
/// the buffer is half-full or full, though, because the DMA
/// controller does not detect those conditions; it sends an
/// interrupt when those specific buffer addresses have been
/// written.
///
/// - If `eager_reads` is enabled in `config`, the UART interrupt
/// is enabled on all data reception and the call will only wait
/// for at least one byte to be available before returning.
///
/// In the first two cases this will result in variable latency due to the
/// buffering effect. For example, if the baudrate is 2400 bps, and
/// the configuration is 8 data bits, no parity bit, and one stop bit,
/// then a byte will be received every ~4.16ms. If the ring buffer is
/// 32 bytes, then a 'wait for bytes' delay may have to wait for 16
/// bytes in the worst case, resulting in a delay (latency) of
/// ~62.46ms for the first byte in the ring buffer. If the sender
/// sends only 6 bytes and then stops, but the buffer was empty when
/// the read function was called, then those bytes may not be returned
/// until ~24.96ms after the first byte was received (time for 5
/// additional bytes plus the 'idle frame' which triggers the 'idle
/// line' condition).
///
/// Applications subject to this latency must be careful if they
/// also apply timeouts during reception, as it may appear (to
/// them) that the sender has stopped sending when it did not. In
/// the example above, a 50ms timeout (12 bytes at 2400bps) might
/// seem to be reasonable to detect that the sender has stopped
/// sending, but would be falsely triggered in the worst-case
/// buffer delay scenario.
///
/// Note: Enabling `eager_reads` with `RingBufferedUartRx` will enable
/// an UART RXNE interrupt, which will cause an interrupt to occur on
/// every received data byte. The data is still copied using DMA, but
/// there is nevertheless additional processing overhead for each byte.
pub struct RingBufferedUartRx<'d> {
    info: &'static Info,
    state: &'static State,
    kernel_clock: Hertz,
    _rx: Option<Flex<'d>>,
    _rts: Option<Flex<'d>>,
    ring_buf: ReadableRingBuffer<'d, u8>,
    diag: RxDiagCounters,
    diag_last_error: Option<RxErrorDiag>,
}

/// Where a `read()` error came from. Diagnostic only: the error returned to
/// the caller is unchanged (every ring error is still reported as
/// [`Error::Overrun`]).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub enum RxErrorSource {
    /// A USART error flag found when the read began, with the receiver
    /// running (CR3.DMAR set).
    UsartAtEntryRunning = 0,
    /// A USART error flag found when the read began, with the receiver
    /// stopped by an earlier error (CR3.DMAR clear): the read returns before
    /// it restarts the receiver.
    UsartAtEntryStopped = 1,
    /// A USART error flag raised while the read waited for data.
    UsartWhileWaiting = 2,
    /// The DMA lapped the read index (ring overrun).
    RingOverrun = 3,
    /// The DMA position went backwards against the read index.
    RingDmaUnsynced = 4,
}

/// Number of [`RxErrorSource`] variants.
pub const RX_ERROR_SOURCES: usize = 5;

/// Register and ring state captured when a `read()` error is detected,
/// before the receiver is stopped. Diagnostic only.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct RxErrorDiag {
    /// Where the error came from.
    pub source: RxErrorSource,
    /// USART ISR (SR on v1/v2), raw. For a USART-flag error, the value the
    /// flag check read (before it cleared the flags); otherwise read live.
    pub isr: u32,
    /// USART CR1, raw.
    pub cr1: u32,
    /// USART CR3, raw (DMAR is bit 6).
    pub cr3: u32,
    /// DMA channel `(SR, CR, BR1, LLI index)` on GPDMA, zeros elsewhere.
    pub dma: (u32, u32, u32, u32),
    /// Ring indices: those of the failed ring read (before its reset) for a
    /// ring error, the current ones otherwise.
    pub ring: crate::dma::RingIndices,
}

/// Cumulative receive-path counters. Diagnostic only.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct RxDiagCounters {
    /// Errors returned by `read()`, by [`RxErrorSource`].
    pub errors: [u32; RX_ERROR_SOURCES],
    /// Times a read found the receiver stopped and restarted it.
    pub auto_restarts: u32,
}

/// Live receive-path state (register reads only). Diagnostic only.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct RxLiveDiag {
    /// USART ISR (SR on v1/v2), raw.
    pub isr: u32,
    /// USART CR1, raw.
    pub cr1: u32,
    /// USART CR3, raw.
    pub cr3: u32,
    /// DMA channel `(SR, CR, BR1, LLI index)` on GPDMA, zeros elsewhere.
    pub dma: (u32, u32, u32, u32),
    /// Ring indices as last synced.
    pub ring: crate::dma::RingIndices,
    /// Cumulative counters.
    pub counters: RxDiagCounters,
}

impl<'d> SetConfig for RingBufferedUartRx<'d> {
    type Config = Config;
    type ConfigError = ConfigError;

    fn set_config(&mut self, config: &Self::Config) -> Result<(), Self::ConfigError> {
        self.set_config(config)
    }
}

impl<'d> UartRx<'d, Async> {
    /// Turn the `UartRx` into a buffered uart which can continously receive in the background
    /// without the possibility of losing bytes. The `dma_buf` is a buffer registered to the
    /// DMA controller, and must be large enough to prevent overflows.
    pub fn into_ring_buffered(mut self, dma_buf: &'d mut [u8]) -> RingBufferedUartRx<'d> {
        assert!(!dma_buf.is_empty() && dma_buf.len() <= 0xFFFF);

        let opts = Default::default();

        // Safety: we forget the struct before this function returns.
        let rx_dma = self.rx_dma.as_mut().unwrap();
        let request = rx_dma.request;
        let rx_dma = unsafe { rx_dma.channel.clone_unchecked() };

        let info = self.info;
        let state = self.state;
        let kernel_clock = self.kernel_clock;
        let ring_buf = unsafe { ReadableRingBuffer::new(rx_dma, request, rdr(info.regs), dma_buf, opts) };
        let rx = unsafe { self.rx.as_ref().map(|x| x.clone_unchecked()) };
        let rts = unsafe { self.rts.as_ref().map(|x| x.clone_unchecked()) };

        info.rcc.increment_stop_refcount();

        // Don't disable the clock
        mem::forget(self);

        RingBufferedUartRx {
            info,
            state,
            kernel_clock,
            _rx: rx,
            _rts: rts,
            ring_buf,
            diag: RxDiagCounters::default(),
            diag_last_error: None,
        }
    }
}

impl<'d> RingBufferedUartRx<'d> {
    /// Reconfigure the driver
    pub fn set_config(&mut self, config: &Config) -> Result<(), ConfigError> {
        self.state
            .eager_reads
            .store(config.eager_reads.unwrap_or(0), Ordering::Relaxed);
        reconfigure(self.info, self.kernel_clock, config)
    }

    /// Configure and start the DMA backed UART receiver
    ///
    /// Note: This is also done automatically by the read functions if
    /// required.
    pub fn start_uart(&mut self) {
        // Clear the buffer so that it is ready to receive data
        compiler_fence(Ordering::SeqCst);
        self.ring_buf.start();
        // `start()` hardware-resets the DMA channel (write position back to the
        // buffer start) but leaves the software ring indices at their pre-error
        // values. `DmaIndex::dma_sync`'s wrap guard then clamps the write index
        // to end-of-ring, and every read returns 0 bytes until a FULL ring of
        // new traffic wraps past it — with sparse polled protocols that is
        // effectively forever. Reset the software indices to match the
        // freshly-reset DMA.
        self.ring_buf.clear();

        let r = self.info.regs;
        // clear all interrupts and DMA Rx Request
        r.cr1().modify(|w| {
            // use RXNE only when returning reads early
            w.set_rxneie(self.state.eager_reads.load(Ordering::Relaxed) > 0);
            // enable parity interrupt if not ParityNone
            w.set_peie(w.pce());
            // enable idle line interrupt
            w.set_idleie(true);
        });
        r.cr3().modify(|w| {
            // enable Error Interrupt: (Frame error, Noise error, Overrun error)
            w.set_eie(true);
            // enable DMA Rx Request
            w.set_dmar(true);
        });
    }

    /// Hard-reset the receive path: stop the DMA, discard everything buffered
    /// (hardware ring position and software indices), clear every latched
    /// error/idle flag, and restart reception from a clean slate.
    ///
    /// Needed when the receiver must be *proactively* cleaned rather than
    /// waiting for a read to observe an error — e.g. at session start against
    /// a device that was powered off while the receiver kept running: its
    /// dead TX holds the line low, which the USART receives as a continuous
    /// break (a 0x00 + framing-error flood into the ring). Note that calling
    /// `start_uart()` on a live channel is not a reliable reset — the channel
    /// must be stopped before it is reconfigured.
    pub fn reset_rx(&mut self) {
        self.stop_uart();
        let r = self.info.regs;
        let sr_val = sr(r).read();
        clear_interrupt_flags(r, sr_val);
        self.start_uart();
    }

    /// Stop DMA backed UART receiver
    fn stop_uart(&mut self) {
        self.ring_buf.request_pause();

        let r = self.info.regs;
        // clear all interrupts and DMA Rx Request
        r.cr1().modify(|w| {
            // disable RXNE interrupt
            w.set_rxneie(false);
            // disable parity interrupt
            w.set_peie(false);
            // disable idle line interrupt
            w.set_idleie(false);
        });
        r.cr3().modify(|w| {
            // disable Error Interrupt: (Frame error, Noise error, Overrun error)
            w.set_eie(false);
            // disable DMA Rx Request
            w.set_dmar(false);
        });

        compiler_fence(Ordering::SeqCst);
    }

    /// (Re-)start DMA and Uart if it is not running (has not been started yet or has failed), and
    /// check for errors in status register. Error flags are checked/cleared first.
    fn start_dma_or_check_errors(&mut self) -> Result<(), Error> {
        let r = self.info.regs;

        let (res, isr) = check_idle_and_errors_sr(r);
        if let Err(e) = res {
            let source = if r.cr3().read().dmar() {
                RxErrorSource::UsartAtEntryRunning
            } else {
                RxErrorSource::UsartAtEntryStopped
            };
            self.diag_record(source, Some(isr), None);
            return Err(e);
        }
        if !r.cr3().read().dmar() {
            self.diag.auto_restarts = self.diag.auto_restarts.wrapping_add(1);
            self.start_uart();
        }
        Ok(())
    }

    /// Record a `read()` error for diagnostics (register reads only).
    fn diag_record(&mut self, source: RxErrorSource, isr: Option<u32>, ring: Option<crate::dma::RingIndices>) {
        let r = self.info.regs;
        let i = source as usize;
        self.diag.errors[i] = self.diag.errors[i].wrapping_add(1);
        self.diag_last_error = Some(RxErrorDiag {
            source,
            isr: isr.unwrap_or_else(|| sr(r).read().0),
            cr1: r.cr1().read().0,
            cr3: r.cr3().read().0,
            dma: self.diag_dma_regs(),
            ring: ring.unwrap_or_else(|| self.ring_buf.ring_indices()),
        });
    }

    #[cfg(gpdma)]
    fn diag_dma_regs(&self) -> (u32, u32, u32, u32) {
        self.ring_buf.channel_regs()
    }

    #[cfg(not(gpdma))]
    fn diag_dma_regs(&self) -> (u32, u32, u32, u32) {
        (0, 0, 0, 0)
    }

    /// Diagnostic: the state captured at the last `read()` error, once.
    pub fn take_last_error_diag(&mut self) -> Option<RxErrorDiag> {
        self.diag_last_error.take()
    }

    /// Diagnostic: cumulative receive-path counters.
    pub fn diag_counters(&self) -> RxDiagCounters {
        self.diag
    }

    /// Diagnostic: live register and ring state (register reads only; the
    /// status register is read, never cleared).
    pub fn diag_live(&self) -> RxLiveDiag {
        let r = self.info.regs;
        RxLiveDiag {
            isr: sr(r).read().0,
            cr1: r.cr1().read().0,
            cr3: r.cr3().read().0,
            dma: self.diag_dma_regs(),
            ring: self.ring_buf.ring_indices(),
            counters: self.diag,
        }
    }

    /// Read bytes that are available in the ring buffer, or wait for
    /// bytes to become available and return them.
    ///
    /// Background reception is started if necessary (if `start_uart()` had
    /// not previously been called, or if an error was detected which
    /// caused background reception to be stopped).
    ///
    /// Background reception is terminated when an error is returned.
    /// It must be started again by calling `start_uart()` or by
    /// calling a read function again.
    pub async fn read(&mut self, buf: &mut [u8]) -> Result<usize, Error> {
        self.start_dma_or_check_errors()?;

        // In half-duplex mode, we need to disable the Transmitter and enable the Receiver
        // since they can't operate simultaneously on the shared line
        let r = self.info.regs;
        if r.cr3().read().hdsel() && r.cr1().read().te() {
            r.cr1().modify(|reg| {
                reg.set_re(true);
                reg.set_te(false);
            });
        }

        loop {
            match self.ring_buf.read(buf) {
                Ok((0, _)) => {}
                Ok((len, _)) => {
                    return Ok(len);
                }
                Err(e) => {
                    let source = match e {
                        crate::dma::ringbuffer::Error::Overrun => RxErrorSource::RingOverrun,
                        crate::dma::ringbuffer::Error::DmaUnsynced => RxErrorSource::RingDmaUnsynced,
                    };
                    let ring = self.ring_buf.take_error_indices();
                    self.diag_record(source, None, ring);
                    self.stop_uart();
                    return Err(Error::Overrun);
                }
            }

            match self.wait_for_data_or_idle().await {
                Ok(_) => {}
                Err((err, isr)) => {
                    self.diag_record(RxErrorSource::UsartWhileWaiting, Some(isr), None);
                    self.stop_uart();
                    return Err(err);
                }
            }
        }
    }

    /// Wait for uart idle or dma half-full or full. An error carries the raw
    /// status register value the flag check read.
    async fn wait_for_data_or_idle(&mut self) -> Result<(), (Error, u32)> {
        compiler_fence(Ordering::SeqCst);

        loop {
            // Future which completes when idle line is detected
            let s = self.state;
            let mut uart_init = false;
            let uart = poll_fn(|cx| {
                s.rx_waker.register(cx.waker());

                compiler_fence(Ordering::SeqCst);

                // We may have been woken by IDLE or, if eager_reads is set, by RXNE.
                // However, DMA will clear RXNE, so we can't check directly, and because
                // the other future borrows `ring_buf`, we can't check `len()` here either.
                // Instead, return from this future and we'll check the length afterwards.
                let eager = s.eager_reads.load(Ordering::Relaxed) > 0;

                let (res, isr) = check_idle_and_errors_sr(self.info.regs);
                let idle = res.map_err(|e| (e, isr))?;
                if idle || (eager && uart_init) {
                    // Idle line is detected, or eager reads is set and some data is available.
                    Poll::Ready(Ok(idle))
                } else {
                    uart_init = true;
                    Poll::Pending
                }
            });

            let mut dma_init = false;
            // Future which completes when the DMA controller indicates it
            // has written to the ring buffer's middle byte, or last byte
            let dma = poll_fn(|cx| {
                self.ring_buf.set_waker(cx.waker());

                let status = match dma_init {
                    false => Poll::Pending,
                    true => Poll::Ready(()),
                };

                dma_init = true;
                status
            });

            match select(uart, dma).await {
                // UART woke with line idle
                Either::Left((Ok(true), _)) => {
                    return Ok(());
                }
                // UART woke without idle or error: word received
                Either::Left((Ok(false), _)) => {
                    let eager = self.state.eager_reads.load(Ordering::Relaxed);
                    if eager > 0 && self.ring_buf.len().unwrap_or(0) >= eager {
                        return Ok(());
                    } else {
                        continue;
                    }
                }
                // UART woke with error
                Either::Left((Err(e), _)) => {
                    return Err(e);
                }
                // DMA woke
                Either::Right(((), _)) => return Ok(()),
            }
        }
    }

    /// Set baudrate
    pub fn set_baudrate(&self, baudrate: u32) -> Result<(), ConfigError> {
        set_baudrate(self.info, self.kernel_clock, baudrate)
    }
}

impl Drop for RingBufferedUartRx<'_> {
    fn drop(&mut self) {
        self.info.rcc.decrement_stop_refcount();
        self.stop_uart();
        super::drop_tx_rx(self.info, self.state);
    }
}

/// Check and clear idle and error interrupts, return true if idle, Err(e) on error
///
/// All flags are read and cleared in a single step, respectively. When more than one flag is set
/// at the same time, all flags will be cleared but only one flag will be reported. So the other
/// flag(s) will gone missing unnoticed. The error flags are checked first, the idle flag last.
///
/// For usart_v1 and usart_v2, all status flags must be handled together anyway because all flags
/// are cleared by a single read to the RDR register.
///
/// Also returns the raw status register value it read (before clearing), for
/// diagnostics.
fn check_idle_and_errors_sr(r: Regs) -> (Result<bool, Error>, u32) {
    // SAFETY: read only and we only use Rx related flags
    let sr = sr(r).read();
    (check_idle_and_errors_from(r, sr), sr.0)
}

#[cfg(any(usart_v1, usart_v2))]
type SrVal = super::regs::Sr;
#[cfg(any(usart_v3, usart_v4))]
type SrVal = super::regs::Isr;

fn check_idle_and_errors_from(r: Regs, sr: SrVal) -> Result<bool, Error> {
    #[cfg(not(any(usart_v3, usart_v4)))]
    unsafe {
        // This read also clears the error and idle interrupt flags on v1 (TODO and v2?)
        rdr(r).read_volatile()
    };
    clear_interrupt_flags(r, sr);

    if sr.pe() {
        Err(Error::Parity)
    } else if sr.fe() {
        Err(Error::Framing)
    } else if sr.ne() {
        Err(Error::Noise)
    } else if sr.ore() {
        Err(Error::Overrun)
    } else {
        r.cr1().modify(|w| w.set_idleie(true));
        Ok(sr.idle())
    }
}

impl embedded_io_async::ErrorType for RingBufferedUartRx<'_> {
    type Error = Error;
}

impl embedded_io_async::Read for RingBufferedUartRx<'_> {
    async fn read(&mut self, buf: &mut [u8]) -> Result<usize, Self::Error> {
        self.read(buf).await
    }
}

impl embedded_hal_nb::serial::Read for RingBufferedUartRx<'_> {
    fn read(&mut self) -> nb::Result<u8, Self::Error> {
        self.start_dma_or_check_errors()?;

        let mut buf = [0u8; 1];
        match self.ring_buf.read(&mut buf) {
            Ok((0, _)) => Err(nb::Error::WouldBlock),
            Ok((len, _)) => {
                assert!(len == 1);
                Ok(buf[0])
            }
            Err(e) => {
                let source = match e {
                    crate::dma::ringbuffer::Error::Overrun => RxErrorSource::RingOverrun,
                    crate::dma::ringbuffer::Error::DmaUnsynced => RxErrorSource::RingDmaUnsynced,
                };
                let ring = self.ring_buf.take_error_indices();
                self.diag_record(source, None, ring);
                self.stop_uart();
                Err(nb::Error::Other(Error::Overrun))
            }
        }
    }
}

impl embedded_hal_nb::serial::ErrorType for RingBufferedUartRx<'_> {
    type Error = Error;
}

impl ReadReady for RingBufferedUartRx<'_> {
    fn read_ready(&mut self) -> Result<bool, Self::Error> {
        let len = self.ring_buf.len().map_err(|e| match e {
            crate::dma::ringbuffer::Error::Overrun => Self::Error::Overrun,
            crate::dma::ringbuffer::Error::DmaUnsynced => {
                error!(
                    "Ringbuffer error: DmaUNsynced, driver implementation is
                    probably bugged please open an issue"
                );
                // we report this as overrun since its recoverable in the same way
                Self::Error::Overrun
            }
        })?;
        Ok(len > 0)
    }
}
