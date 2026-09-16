#[cfg(not(stm32n6))]
use core::sync::atomic::{AtomicBool, Ordering, compiler_fence};

#[cfg(not(stm32n6))]
use crate::pac::common::{RW, Reg};
#[cfg(backup_sram)]
use crate::pac::pwr::vals::Retention;
pub use crate::pac::rcc::vals::Rtcsel as RtcClockSource;
use crate::time::Hertz;

#[cfg(any(stm32f0, stm32f1, stm32f3))]
pub const LSI_FREQ: Hertz = Hertz(40_000);
#[cfg(not(any(stm32f0, stm32f1, stm32f3)))]
pub const LSI_FREQ: Hertz = Hertz(32_000);

/// Set when the LSE oscillator was requested but did not report `LSERDY`
/// within [`LSE_STARTUP_TIMEOUT_POLLS`] × [`LSE_STARTUP_POLL_CYCLES`] core
/// cycles during [`LsConfig::init`]. In that case the LSE is switched back
/// off and, if it was the requested RTC source, the RTC is clocked from the
/// LSI instead so that boot can proceed. Read it with [`lse_start_failed`].
#[cfg(not(stm32n6))]
static LSE_START_FAILED: AtomicBool = AtomicBool::new(false);

/// Core cycles spent in `cortex_m::asm::delay` between two `LSERDY` polls.
#[cfg(not(stm32n6))]
const LSE_STARTUP_POLL_CYCLES: u32 = 1_000;

/// Number of `LSERDY` polls before giving up on the LSE. With
/// [`LSE_STARTUP_POLL_CYCLES`] this is ~192 M core cycles: ≈3 s on a 64 MHz
/// HSI, which is the system clock while `LsConfig::init` runs on most
/// families (the PLL is switched in afterwards). A 32.768 kHz crystal needs
/// 0.5–2 s to start; a crystal that is still silent after this bound is
/// missing, broken or badly loaded. This is a *bound* on the stall, not a
/// calibrated duration — on a slower core clock it simply waits longer.
#[cfg(not(stm32n6))]
const LSE_STARTUP_TIMEOUT_POLLS: u32 = 192_000;

/// Returns `true` if the LSE was requested in [`LsConfig`] but failed to start
/// during [`LsConfig::init`], so the RTC is running on the LSI fallback.
#[cfg(not(stm32n6))]
pub fn lse_start_failed() -> bool {
    LSE_START_FAILED.load(Ordering::Relaxed)
}

/// Poll `ready` until it returns `true` or the LSE startup bound elapses.
#[cfg(not(stm32n6))]
fn wait_lse_ready(ready: impl Fn() -> bool) -> bool {
    for _ in 0..LSE_STARTUP_TIMEOUT_POLLS {
        if ready() {
            return true;
        }
        cortex_m::asm::delay(LSE_STARTUP_POLL_CYCLES);
    }
    ready()
}

/// Switch the LSI on and wait for it to be ready. The LSI is an internal RC
/// oscillator: it always starts, so this wait is unbounded like the rest of
/// the RCC bring-up.
#[cfg(not(stm32n6))]
fn enable_lsi() {
    #[cfg(any(stm32u5, stm32h5, stm32wba))]
    let csr = crate::pac::RCC.bdcr();
    #[cfg(not(any(stm32u5, stm32h5, stm32wba, stm32c0)))]
    let csr = crate::pac::RCC.csr();
    #[cfg(stm32c0)]
    let csr = crate::pac::RCC.csr2();

    #[cfg(not(any(rcc_wb, rcc_wba)))]
    csr.modify(|w| w.set_lsion(true));

    #[cfg(any(rcc_wb, rcc_wba))]
    csr.modify(|w| w.set_lsi1on(true));

    #[cfg(not(any(rcc_wb, rcc_wba)))]
    while !csr.read().lsirdy() {}

    #[cfg(any(rcc_wb, rcc_wba))]
    while !csr.read().lsi1rdy() {}
}

#[allow(dead_code)]
#[derive(Clone, Copy)]
pub enum LseMode {
    Oscillator(LseDrive),
    Bypass,
}

#[derive(Clone, Copy)]
pub struct LseConfig {
    pub frequency: Hertz,
    pub mode: LseMode,
    /// If peripherals other than RTC/TAMP or RCC functions need the lse this bit must be set
    #[cfg(any(rcc_l5, rcc_u5, rcc_u3, rcc_wle, rcc_wl5, rcc_wba))]
    pub peripherals_clocked: bool,
}

#[allow(dead_code)]
#[derive(Default, Clone, Copy)]
pub enum LseDrive {
    #[cfg(not(stm32h5))] // ES0565: LSE Low drive mode is not functional
    Low = 0,
    MediumLow = 0x01,
    #[default]
    MediumHigh = 0x02,
    High = 0x03,
}

// All families but these have the LSEDRV register
#[cfg(not(any(rcc_f1, rcc_f1cl, rcc_f100, rcc_f2, rcc_f4, rcc_f410, rcc_l1)))]
impl From<LseDrive> for crate::pac::rcc::vals::Lsedrv {
    fn from(value: LseDrive) -> Self {
        use crate::pac::rcc::vals::Lsedrv;

        match value {
            #[cfg(not(stm32h5))] // ES0565: LSE Low drive mode is not functional
            LseDrive::Low => Lsedrv::LOW,
            LseDrive::MediumLow => Lsedrv::MEDIUM_LOW,
            LseDrive::MediumHigh => Lsedrv::MEDIUM_HIGH,
            LseDrive::High => Lsedrv::HIGH,
        }
    }
}

#[cfg(not(any(rtc_v2_l0, rtc_v2_l1, stm32c0, stm32n6)))]
type Bdcr = crate::pac::rcc::regs::Bdcr;
#[cfg(any(rtc_v2_l0, rtc_v2_l1))]
type Bdcr = crate::pac::rcc::regs::Csr;
#[cfg(any(stm32c0))]
type Bdcr = crate::pac::rcc::regs::Csr1;

#[cfg(any(stm32c0))]
fn unlock() {}

#[cfg(not(any(stm32c0, stm32n6)))]
fn unlock() {
    #[cfg(any(stm32f0, stm32f1, stm32f2, stm32f3, stm32l0, stm32l1))]
    let cr = crate::pac::PWR.cr();
    #[cfg(not(any(
        stm32f0, stm32f1, stm32f2, stm32f3, stm32l0, stm32l1, stm32u5, stm32u3, stm32h5, stm32wba, stm32n6
    )))]
    let cr = crate::pac::PWR.cr1();
    #[cfg(any(stm32u5, stm32u3, stm32h5, stm32wba, stm32n6))]
    let cr = crate::pac::PWR.dbpcr();

    cr.modify(|w| w.set_dbp(true));
    while !cr.read().dbp() {}
}

#[cfg(not(stm32n6))]
fn bdcr() -> Reg<Bdcr, RW> {
    #[cfg(any(rtc_v2_l0, rtc_v2_l1))]
    return crate::pac::RCC.csr();
    #[cfg(not(any(rtc_v2_l0, rtc_v2_l1, stm32c0)))]
    return crate::pac::RCC.bdcr();
    #[cfg(any(stm32c0))]
    return crate::pac::RCC.csr1();
}

#[derive(Clone, Copy)]
pub struct LsConfig {
    pub rtc: RtcClockSource,
    pub lsi: bool,
    pub lse: Option<LseConfig>,
    #[cfg(backup_sram)]
    pub enable_backup_sram: bool,
}

impl LsConfig {
    /// Creates an [`LsConfig`] using the LSI when possible.
    pub const fn new() -> Self {
        // on L5, just the fact that LSI is enabled makes things crash.
        // TODO: investigate.

        #[cfg(not(stm32l5))]
        return Self::default_lsi();
        #[cfg(stm32l5)]
        return Self::off();
    }

    pub const fn default_lse() -> Self {
        Self {
            rtc: RtcClockSource::LSE,
            lse: Some(LseConfig {
                frequency: Hertz(32_768),
                mode: LseMode::Oscillator(LseDrive::MediumHigh),
                #[cfg(any(rcc_l5, rcc_u5, rcc_u3, rcc_wle, rcc_wl5, rcc_wba))]
                peripherals_clocked: false,
            }),
            lsi: false,
            #[cfg(backup_sram)]
            enable_backup_sram: false,
        }
    }

    pub const fn default_lsi() -> Self {
        Self {
            rtc: RtcClockSource::LSI,
            lsi: true,
            lse: None,
            #[cfg(backup_sram)]
            enable_backup_sram: false,
        }
    }

    pub const fn off() -> Self {
        Self {
            rtc: RtcClockSource::DISABLE,
            lsi: false,
            lse: None,
            #[cfg(backup_sram)]
            enable_backup_sram: false,
        }
    }
}

impl Default for LsConfig {
    fn default() -> Self {
        Self::new()
    }
}

impl LsConfig {
    #[cfg(not(stm32n6))]
    pub(crate) fn init(&self) -> Option<Hertz> {
        let rtc_clk = match self.rtc {
            RtcClockSource::LSI => {
                assert!(self.lsi);
                Some(LSI_FREQ)
            }
            RtcClockSource::LSE => Some(self.lse.as_ref().unwrap().frequency),
            RtcClockSource::DISABLE => None,
            _ => todo!(),
        };

        // Both may be downgraded below if the LSE fails to start.
        #[allow(unused_mut)]
        let (mut lse_en, lse_byp, lse_drv) = match &self.lse {
            Some(c) => match c.mode {
                LseMode::Oscillator(lse_drv) => (true, false, Some(lse_drv)),
                LseMode::Bypass => (true, true, None),
            },
            None => (false, false, None),
        };
        #[cfg(any(rcc_l5, rcc_u5, rcc_wle, rcc_wl5, rcc_wba))]
        let lse_sysen = if let Some(lse) = self.lse {
            Some(lse.peripherals_clocked)
        } else {
            None
        };
        #[cfg(rcc_u0)]
        let lse_sysen = Some(lse_en);

        _ = lse_drv; // not all chips have it.

        #[allow(unused_mut)]
        let mut rtc = self.rtc;

        // Disable backup domain write protection
        unlock();

        if self.lsi {
            enable_lsi();
        }

        // Enable backup regulator for peristent battery backed sram
        #[cfg(backup_sram)]
        {
            unsafe { super::BKSRAM_RETAINED = crate::pac::PWR.bdcr().read().bren() == Retention::PRESERVED };

            crate::pac::PWR.bdcr().modify(|w| {
                w.set_bren(match self.enable_backup_sram {
                    true => Retention::PRESERVED,
                    false => Retention::LOST,
                });
            });

            // Wait for backup regulator voltage to stabilize
            while self.enable_backup_sram && !crate::pac::PWR.bdsr().read().brrdy() {}
        }

        // backup domain configuration (LSEON, RTCEN, RTCSEL) is kept across resets.
        // once set, changing it requires a backup domain reset.
        // first check if the configuration matches what we want.
        // N6 has all the fields spread across multiple registers under RCC.

        // check if it's already enabled and in the source we want.
        #[cfg(not(rcc_n6))]
        let reg = bdcr().read();
        #[cfg(rcc_n6)]
        let reg = crate::pac::RCC.cr().read();
        #[cfg(rcc_n6)]
        let apb4lenr = crate::pac::RCC.apb4lenr().read();
        #[cfg(rcc_n6)]
        let ccipr7 = crate::pac::RCC.ccipr7().read();
        #[cfg(rcc_n6)]
        let lsecfgr = crate::pac::RCC.lsecfgr().read();

        let mut ok = true;
        #[cfg(not(rcc_n6))]
        {
            ok &= reg.rtcsel() == self.rtc;
        }
        #[cfg(rcc_n6)]
        {
            ok &= ccipr7.rtcsel() == self.rtc;
        }
        #[cfg(not(any(rcc_wba, rcc_n6)))]
        {
            ok &= reg.rtcen() == (self.rtc != RtcClockSource::DISABLE);
        }
        #[cfg(rcc_n6)]
        {
            ok &= apb4lenr.rtcen() == (self.rtc != RtcClockSource::DISABLE);
        }
        ok &= reg.lseon() == lse_en;
        #[cfg(not(rcc_n6))]
        {
            ok &= reg.lsebyp() == lse_byp;
        }
        #[cfg(rcc_n6)]
        {
            ok &= lsecfgr.lsebyp() == lse_byp;
        }
        #[cfg(any(rcc_l5, rcc_u5, rcc_wle, rcc_wl5, rcc_wba, rcc_u0))]
        if let Some(lse_sysen) = lse_sysen
            && !lse_sysen
        {
            ok &= !reg.lsesysen();
        }
        #[cfg(not(any(rcc_f1, rcc_f1cl, rcc_f100, rcc_f2, rcc_f4, rcc_f410, rcc_l1, rcc_n6)))]
        if let Some(lse_drv) = lse_drv {
            ok &= reg.lsedrv() == lse_drv.into();
        }
        #[cfg(rcc_n6)]
        if let Some(lse_drv) = lse_drv {
            ok &= lsecfgr.lsedrv() == lse_drv.into();
        }

        // After a power-on reset LSESYSEN will be set to 0
        // even if VBAT was present and kept the RTC running
        #[cfg(any(rcc_l5, rcc_u5, rcc_wle, rcc_wl5, rcc_wba, rcc_u0))]
        if ok
            && let Some(lse_sysen) = lse_sysen
            && lse_sysen
        {
            bdcr().modify(|w| {
                w.set_lsesysen(true);
            });

            while !bdcr().read().lsesysrdy() {}
        }

        // if configuration is OK, we're done.
        if ok {
            trace!("BDCR ok: {:08x}", bdcr().read().0);
            return rtc_clk;
        }

        // If not OK, reset backup domain and configure it.
        #[cfg(not(any(rcc_l0, rcc_l0_v2, rcc_l1, stm32h5, stm32h7rs, stm32c0, stm32n6)))]
        {
            bdcr().modify(|w| w.set_bdrst(true));
            bdcr().modify(|w| w.set_bdrst(false));
        }
        // H5 has a terrible, terrible errata: 'SRAM2 is erased when the backup domain is reset'
        // pending a more sane sane way to handle this, just don't reset BD for now.
        // This means the RTCSEL write below will have no effect, only if it has already been written
        // after last power-on. Since it's uncommon to dynamically change RTCSEL, this is better than
        // letting half our RAM go magically *poof*.
        // STM32H503CB/EB/KB/RB device errata - 2.2.8 SRAM2 unduly erased upon a backup domain reset
        // STM32H562xx/563xx/573xx device errata - 2.2.14 SRAM2 is erased when the backup domain is reset
        //#[cfg(any(stm32h5, stm32h7rs))]
        #[cfg(any(stm32h7rs, stm32n6))]
        {
            bdcr().modify(|w| w.set_vswrst(true));
            bdcr().modify(|w| w.set_vswrst(false));
        }
        #[cfg(any(stm32c0, stm32l0))]
        {
            bdcr().modify(|w| w.set_rtcrst(true));
            bdcr().modify(|w| w.set_rtcrst(false));
        }

        if lse_en {
            #[cfg(not(rcc_n6))]
            {
                bdcr().modify(|w| {
                    #[cfg(not(any(rcc_f1, rcc_f1cl, rcc_f100, rcc_f2, rcc_f4, rcc_f410, rcc_l1)))]
                    if let Some(lse_drv) = lse_drv {
                        w.set_lsedrv(lse_drv.into());
                    }
                    w.set_lsebyp(lse_byp);
                    w.set_lseon(true);
                });

                if !wait_lse_ready(|| bdcr().read().lserdy()) {
                    // The crystal never came up. Do not hang the boot on it:
                    // switch the oscillator back off, remember the failure and
                    // keep the RTC alive on the LSI if it was meant to use the LSE.
                    bdcr().modify(|w| w.set_lseon(false));
                    LSE_START_FAILED.store(true, Ordering::Relaxed);
                    lse_en = false;
                    if rtc == RtcClockSource::LSE {
                        if !self.lsi {
                            enable_lsi();
                        }
                        rtc = RtcClockSource::LSI;
                    }
                }
            }
            #[cfg(rcc_n6)]
            {
                crate::pac::RCC.lsecfgr().modify(|w| {
                    if let Some(lse_drv) = lse_drv {
                        w.set_lsedrv(lse_drv.into());
                    }
                    w.set_lsebyp(lse_byp);
                });
                crate::pac::RCC.cr().modify(|w| w.set_lseon(true));

                while !crate::pac::RCC.sr().read().lserdy() {}
            }

            // Skipped when the LSE failed to start above: LSESYSRDY would never come.
            #[cfg(any(rcc_l5, rcc_u5, rcc_wle, rcc_wl5, rcc_wba, rcc_u0))]
            if lse_en && let Some(lse_sysen) = lse_sysen {
                bdcr().modify(|w| {
                    w.set_lsesysen(lse_sysen);
                });

                if lse_sysen {
                    while !bdcr().read().lsesysrdy() {}
                }
            }
            // Families without LSESYSEN never read the downgraded flag.
            let _ = lse_en;
        }

        if rtc != RtcClockSource::DISABLE {
            #[cfg(not(rcc_n6))]
            bdcr().modify(|w| {
                #[cfg(any(rtc_v2_h7, rtc_v2_l4, rtc_v2_wb, rtc_v3_base, rtc_v3_u5))]
                assert!(!w.lsecsson(), "RTC is not compatible with LSE CSS, yet.");

                #[cfg(not(rcc_wba))]
                w.set_rtcen(true);
                w.set_rtcsel(rtc);
            });

            #[cfg(rcc_n6)]
            {
                crate::pac::RCC.ccipr7().modify(|w| w.set_rtcsel(rtc));
                crate::pac::RCC.apb4lenr().modify(|w| w.set_rtcen(true))
            }
        }

        trace!("BDCR configured: {:08x}", bdcr().read().0);

        compiler_fence(Ordering::SeqCst);

        // Report the source the RTC is *actually* clocked from, not the one
        // that was requested: the LSE may have been downgraded to the LSI
        // above, and on parts where the backup domain is not reset (H5, see
        // the errata note) RTCSEL is write-once until the next power-on, so
        // the write above may have been ignored. Handing the wrong frequency
        // to the RTC driver would silently mis-program its prescalers.
        #[cfg(not(rcc_n6))]
        let rtc_clk = match bdcr().read().rtcsel() {
            RtcClockSource::LSE => self.lse.as_ref().map(|c| c.frequency),
            RtcClockSource::LSI => Some(LSI_FREQ),
            RtcClockSource::DISABLE => None,
            _ => rtc_clk,
        };

        rtc_clk
    }
}
