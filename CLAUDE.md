# Project instructions

Write clean, maintainable, functional code using the simplest effective solution. Favor clear data flow, small reusable helpers, and behavior that is easy to reason about. Avoid speculative features, configurability, and abstractions that are not required by the current task.

Before implementing, inspect the relevant code, tests, and documentation. Surface assumptions or competing interpretations when they would materially affect the result. Ask for clarification when necessary, and recommend a simpler approach when one exists.

Keep changes surgical. Touch only what the task requires, preserve established conventions, and avoid unrelated refactors, formatting changes, or comment churn. Remove code made obsolete by your changes, but mention unrelated problems instead of fixing them without permission.

For non-trivial work, define observable success criteria before implementation. Reproduce bugs with a regression test when practical, preserve a passing baseline during refactors, and verify the result with checks proportional to the change's risk. Continue until the stated criteria are satisfied or a concrete blocker is identified.

Be simple, clear, and concise in conversation. On completion, summarize the behavior changed, verification performed, and any remaining risks or tradeoffs.

See REFERENCE.md for the canonical technical reference. Update it when implemented architecture, interfaces, invariants, or operational behavior change; do not add speculative plans. You should always be updating REFERENCE.md as you make changes to the repo. Optimize for clarity for humans and coding agents in the reference documents.

When the user says git acp, they mean git add, commit, and push the current changes. Use git push origin main for the push step.
