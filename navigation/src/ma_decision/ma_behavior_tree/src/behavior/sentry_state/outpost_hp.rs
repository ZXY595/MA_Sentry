use std::sync::atomic::{AtomicU16, Ordering};

#[derive(Debug, Default)]
pub struct OutpostHpState(AtomicU16);

impl super::State for OutpostHpState {
    type StateLoad = u16;
    type StateStore = u16;

    fn load(&self) -> Self::StateLoad {
        self.0.load(Ordering::Acquire)
    }

    fn store(&self, state: Self::StateStore) {
        self.0.store(state, Ordering::Release)
    }
}
