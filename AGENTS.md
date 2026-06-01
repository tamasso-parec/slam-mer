<!-- Copyright (C) 2026 Mitsubishi Electric Research Laboratories (MERL) -->
<!-- SPDX-License-Identifier: AGPL-3.0-or-later -->

Use English for implementation and its comments.
Follow the conventions in the latest files.

## C++ naming conventions

- **Parameters**: trailing underscore — `path_`, `grayscale_`, `keyframeId_`
- **Private members**: leading underscore — `_data`, `_rows`, `_step`
- **Private methods**: double leading underscore — `__start_thread()`, `__optimize()`
- **Public methods / free functions**: `snake_case` — `get_point_ids()`, `read_png()`
- **Doc comments**: `///` style with `@brief` / `@param` / `@return`
