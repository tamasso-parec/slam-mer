# Copyright (C) 2026 Mitsubishi Electric Research Laboratories (MERL)
#
# SPDX-License-Identifier: AGPL-3.0-or-later

import os

project = "SLAM-MER"
copyright = "2026, Mitsubishi Electric Research Laboratories (MERL)"
author = "Pedro Miraldo, Valter Piedade, Masashi Yamazaki, and Lalit Manam"
release = "1.0"
version = "1.0"

fastDocsMode = os.environ.get("SLAM_MER_DOCS_FAST", "0") == "1"
extensions = ["sphinx.ext.githubpages"]
if not fastDocsMode:
    extensions.extend(
        [
            "breathe",
            "exhale",
        ]
    )

templates_path = ["_templates"]
exclude_patterns = ["_build", "Thumbs.db", ".DS_Store"]
if fastDocsMode:
    # Skip generated API pages for fast local documentation rebuilds.
    exclude_patterns.extend(["api", "api/**"])
rst_prolog = """
.. role:: u
      :class: underline
.. role:: iu
      :class: underline-initial
"""
suppress_warnings = [
    "cpp",
    "toc.not_included",
]
if fastDocsMode:
    suppress_warnings.append("toc.excluded")


# -- Breathe and Exhale Configuration -----------------------------------------
if not fastDocsMode:
    from exhale import utils as exhale_utils

    breathe_projects = {"SLAM-MER": "./doxygen/_docs/xml"}
    breathe_default_project = "SLAM-MER"

    def _exhale_specs(kind):
        # Render class and struct members (methods and attributes) on API.
        if kind in {"class", "struct"}:
            return [
                ":members:",
                ":protected-members:",
                ":private-members:",
                ":undoc-members:",
            ]
        return []

    exhale_args = {
        # Configure generated Breathe directives by entity kind.
        "customSpecificationsMapping":
            exhale_utils.makeCustomSpecificationsMapping(
                _exhale_specs
            ),
        # These arguments are required
        "containmentFolder": "./api",
        "rootFileName": "library_root.rst",
        "rootFileTitle": "Library API",
        "doxygenStripFromPath": ".",
        # Suggested optional arguments
        "createTreeView":        False,
        # TIP: if using the sphinx-bootstrap-theme, you need
        # "treeViewIsBootstrap": True,
    }
# Tell sphinx what the primary language being documented is.
primary_domain = "cpp"
# Tell sphinx what the pygments highlight language should be.
highlight_language = "cpp"
pygments_style = "sphinx"

html_theme = "sphinx_rtd_theme"
html_static_path = ["_static"]
html_theme_options = {
    "style_external_links": True,
    "style_nav_header_background": "#000000",
    "logo_only": False,
    "collapse_navigation": False,
    "sticky_navigation": True,
    "navigation_depth": 2,
    "includehidden": True,
    "titles_only": False,
}

html_logo = "figures/small.jpg"
