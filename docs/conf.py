from pygments.lexers.special import TextLexer
from sphinx.highlighting import lexers

project = "motion-spec-dsl"
author = "secorolab"
copyright = "secorolab"

extensions = ["myst_parser"]
myst_enable_extensions = ["colon_fence", "deflist"]

html_theme = "furo"
html_title = "motion-spec-dsl"

exclude_patterns = ["_build", "Thumbs.db", ".DS_Store"]

# `.robmot` is not a real Pygments lexer; render those fences as plain text
# instead of emitting a highlighting warning.
lexers["robmot"] = TextLexer()
