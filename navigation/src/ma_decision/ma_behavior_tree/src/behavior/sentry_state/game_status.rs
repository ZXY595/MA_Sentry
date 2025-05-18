use std::sync::atomic::{AtomicU8, Ordering};

#[derive(Debug, Default)]
pub struct GameState(AtomicU8);

impl super::State for GameState {
    type StateLoad = GameStatus;
    type StateStore = u8;

    fn load(&self) -> Self::StateLoad {
        self.0.load(Ordering::Acquire).into()
    }

    fn store(&self, state: Self::StateStore) {
        self.0.store(state, Ordering::Release)
    }
}

#[derive(Clone, Debug, PartialEq, Eq)]
#[repr(u8)]
pub enum GameStatus {
    Idle = 0,
    Prepare = 3,
    InGame = 4,
    Unknown,
}

impl From<u8> for GameStatus {
    fn from(status: u8) -> Self {
        match status {
            0 => GameStatus::Idle,
            3 => GameStatus::Prepare,
            4 => GameStatus::InGame,
            _ => GameStatus::Unknown,
        }
    }
}
