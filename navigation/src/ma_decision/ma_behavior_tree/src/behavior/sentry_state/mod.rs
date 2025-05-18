use smol::future::yield_now;

mod game_status;
mod hp;
mod outpost_hp;
mod spin;
pub use game_status::GameStatus;
pub use hp::HpState;
#[expect(unused_imports)]
pub use outpost_hp::OutpostHpState;
#[expect(unused_imports)]
pub use spin::SpinState;

pub trait State {
    type StateLoad;
    type StateStore;

    fn load(&self) -> Self::StateLoad;

    fn store(&self, state: Self::StateStore);

    async fn wait_until(&self, mut predicate: impl FnMut(Self::StateLoad) -> bool) {
        while predicate(self.load()) {
            yield_now().await;
        }
    }
}

macro_rules! derive_sentry_state {
     ($(#[$attr:meta])*
     $vis:vis struct $name:ident {
         $($field_vis:vis $field:ident: $ty:ty),*
         $(
             ,#[state_igore]
             $($field_i_vis:vis $field_i:ident: $ty_i:ty),*$(,)?
         )?
     }) => {
        $(#[$attr])*
        $vis struct $name {
            $($field_vis $field: $ty),*
            $(,$($field_i_vis $field_i: $ty_i),*)?
        }
        impl State for $name {
            type StateLoad = ($(<$ty as State>::StateLoad),*);
            type StateStore = ($(<$ty as State>::StateStore),*);

            fn load(&self) -> Self::StateLoad {
                ($(self.$field.load(),)*)
            }

            fn store(&self, state: Self::StateStore) {
                let ($($field,)*) = state;
                $(self.$field.store($field);)*
            }
        }
    }
}

derive_sentry_state!(
    #[derive(Debug, Default)]
    pub struct SentryStates {
        pub(crate) game_status: game_status::GameState,
        pub(crate) hp: hp::HpState,
        pub(crate) outpost_hp: outpost_hp::OutpostHpState,
        #[state_igore]
        pub(crate) spin: spin::SpinState,
    }
);
