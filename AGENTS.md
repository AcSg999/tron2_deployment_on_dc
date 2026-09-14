# Working agreements

Maintain English `.md` and Simplified Chinese `.zh-CN.md` versions together for authored Markdown. Preserve third-party license/notice text and provenance.

The application scope is object-aware wrist pregrasp, ending before contact. Do not add gripper commands or a duplicate simulation viewer. Keep imports and service startup free of hardware connections; use explicit mock mode in automated tests. Hardware command execution requires the existing profile, exact-plan review and supervision gates.

Run `python -m pytest -q` and the packaged mock demo when changing the workflow. Keep captured data, local machine profiles, generated plans, logs and secrets out of Git. Do not import the archived `dexpipe` source tree.
