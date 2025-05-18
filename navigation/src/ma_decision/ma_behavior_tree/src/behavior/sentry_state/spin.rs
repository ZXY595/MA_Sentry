use std::sync::atomic::{AtomicBool, Ordering};

#[derive(Debug, Default)]
pub struct SpinState(AtomicBool);

impl super::State for SpinState {
    type StateLoad = bool;
    type StateStore = bool;

    fn load(&self) -> Self::StateLoad {
        self.0.load(Ordering::Acquire)
    }

    fn store(&self, state: Self::StateStore) {
        self.0.store(state, Ordering::Release)
    }
}
