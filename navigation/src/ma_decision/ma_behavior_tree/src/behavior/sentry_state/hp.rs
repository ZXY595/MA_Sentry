use std::sync::atomic::{AtomicU16, Ordering};

#[derive(Debug, Default)]
pub struct HpState(AtomicU16);

impl HpState {
    pub const FULL: u16 = 400;
}

impl super::State for HpState {
    type StateLoad = u16;
    type StateStore = u16;

    fn load(&self) -> Self::StateLoad {
        self.0.load(Ordering::Acquire)
    }

    fn store(&self, state: Self::StateStore) {
        self.0.store(state, Ordering::Release)
    }
}
