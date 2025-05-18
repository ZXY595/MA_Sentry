use crate::behavior::{SentryCtx, sentry_state::SentryStates};

#[test]
fn whole_tree() {}

impl SentryCtx {
    fn new_test() -> Self {
        let states = SentryStates::default();
        Self {
            nav_clinet: todo!(),
            states,
        }
    }
}
