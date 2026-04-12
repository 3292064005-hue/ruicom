"""Pure contract validation for managed vendor sidecar feedback wiring."""

from __future__ import annotations

from dataclasses import dataclass

from .common import ConfigurationError


@dataclass(frozen=True)
class VendorSidecarFeedbackContract:
    """Managed-sidecar declaration of the explicit feedback source.

    Attributes:
        require_explicit_feedback_source: Whether the managed sidecar must refuse
            startup unless one explicit feedback source mode is declared.
        vendor_execution_feedback_topic: Non-empty when the repository adapter
            should subscribe to one vendor-native bool feedback topic and
            republish it onto the canonical bridge ingress.
        upstream_feedback_topic: Canonical bridge ingress topic that ultimately
            carries explicit execution feedback.
        native_feedback_source_declared: Operator acknowledgement that the
            vendor graph itself already publishes the canonical
            ``upstream_feedback_topic`` and therefore does not need the in-repo
            adapter.
    """

    require_explicit_feedback_source: bool
    vendor_execution_feedback_topic: str
    upstream_feedback_topic: str
    native_feedback_source_declared: bool


@dataclass(frozen=True)
class VendorSidecarFeedbackDecision:
    """Normalized validation result for the managed sidecar feedback contract."""

    source_mode: str
    vendor_execution_feedback_topic: str
    upstream_feedback_topic: str


def validate_vendor_sidecar_feedback_contract(contract: VendorSidecarFeedbackContract) -> VendorSidecarFeedbackDecision:
    """Validate how a managed sidecar obtains explicit chassis execution feedback.

    Args:
        contract: Feedback source declaration gathered from launch parameters.

    Returns:
        VendorSidecarFeedbackDecision describing the resolved source mode.

    Raises:
        ConfigurationError: If the declaration is incomplete, ambiguous or would
            create a self-loop between the adapter input and output topics.

    Boundary behavior:
        Exactly one source mode is accepted when explicit feedback is required:
        either ``adapter`` (repository adapter subscribes to one vendor topic)
        or ``native`` (operator explicitly declares that the vendor graph
        already publishes the canonical upstream feedback topic).
    """
    upstream_feedback_topic = str(contract.upstream_feedback_topic or '').strip()
    vendor_execution_feedback_topic = str(contract.vendor_execution_feedback_topic or '').strip()
    native_feedback_source_declared = bool(contract.native_feedback_source_declared)
    require_explicit_feedback_source = bool(contract.require_explicit_feedback_source)

    if not upstream_feedback_topic:
        raise ConfigurationError('managed vendor sidecar requires non-empty upstream_feedback_topic')

    adapter_mode = bool(vendor_execution_feedback_topic)
    native_mode = native_feedback_source_declared

    if adapter_mode and native_mode:
        raise ConfigurationError(
            'managed vendor sidecar feedback source is ambiguous: set vendor_execution_feedback_topic or native_feedback_source_declared=true, not both'
        )
    if adapter_mode and vendor_execution_feedback_topic == upstream_feedback_topic:
        raise ConfigurationError(
            'vendor_execution_feedback_topic must differ from upstream_feedback_topic to avoid adapter self-loop'
        )
    if require_explicit_feedback_source and not adapter_mode and not native_mode:
        raise ConfigurationError(
            'managed vendor sidecar requires declared explicit feedback source: provide vendor_execution_feedback_topic or set native_feedback_source_declared=true'
        )
    if adapter_mode:
        return VendorSidecarFeedbackDecision(
            source_mode='adapter',
            vendor_execution_feedback_topic=vendor_execution_feedback_topic,
            upstream_feedback_topic=upstream_feedback_topic,
        )
    if native_mode:
        return VendorSidecarFeedbackDecision(
            source_mode='native',
            vendor_execution_feedback_topic='',
            upstream_feedback_topic=upstream_feedback_topic,
        )
    return VendorSidecarFeedbackDecision(
        source_mode='optional_undeclared',
        vendor_execution_feedback_topic='',
        upstream_feedback_topic=upstream_feedback_topic,
    )
