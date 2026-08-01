# SPDX-License-Identifier: MPL-2.0
# SPDX-FileCopyrightText: 2026 SECORO AG (secoro.uni-bremen.de)
# Author: Vamsi Kalagaturu
"""textX entry-point descriptors for the motion-spec language."""

from textx import GeneratorDesc, LanguageDesc

from motion_spec_dsl.gens import _gen_graph
from motion_spec_dsl.langs import motion_spec_metamodel

motion_spec_lang = LanguageDesc(
    "motion_spec_dsl",
    pattern="*.robmot",
    description="Language for guarded robot motion specifications",
    metamodel=motion_spec_metamodel,
)

motion_spec_gen = GeneratorDesc(
    language="motion_spec_dsl",
    target="jsonld",
    description="Generate JSON-LD from a motion specification",
    generator=_gen_graph,
)
