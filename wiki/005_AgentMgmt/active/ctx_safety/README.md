---
status: active
---

# ctx_safety

No hardware safety rules yet — this project is a pure offline simulator
with no connection to a real robot controller. If that changes (e.g. this
code is later used to drive a real FR5 over its SDK), safety rules go here
**before** any motion-control code is written, following the postmortem
format from the wiki construction guide: rule, source event, why it's
necessary.
