"""Backward-compatible facade over split common modules.

Existing callers can continue importing from ``common`` while newer code imports
from narrower modules such as ``domain_models`` or ``schema_utils``.
"""

from __future__ import annotations

from .artifact_projection import *  # noqa: F401,F403
from .domain_models import *  # noqa: F401,F403
from .geometry_utils import *  # noqa: F401,F403
from .manifest_utils import *  # noqa: F401,F403
from .route_utils import *  # noqa: F401,F403
from .runtime_paths import *  # noqa: F401,F403
from .schema_utils import *  # noqa: F401,F403
