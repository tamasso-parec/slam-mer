<!-- Copyright (C) 2026 Mitsubishi Electric Research Laboratories (MERL) -->
<!-- SPDX-License-Identifier: AGPL-3.0-or-later -->

## Commit message conventions

Format: `[<branch-name>] <type>(<scope>): <short description>`

- **Branch prefix** — wrap the branch name in `[...]` at the start
- **Type** — conventional commit type
- **Scope** — the component or area affected (e.g. `depth map`, `slam`, `types`)
- **Description** — short, imperative sentence; no trailing period

### Body with bullet points

Add a bullet-point body when the commit touches multiple things or needs context.
Leave a blank line between the subject and the body.

### Example

```txt
[feature/image_t] feat(depth map): add support of depth map type to image_t

- add depth_map_t struct to types/
- extend image_t to hold an optional depth map
- update serialization in image_io.hpp
```
