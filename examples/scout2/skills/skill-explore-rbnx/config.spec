# Runtime config accepted by the Explore skill.
#
# The package consumes no package-level `config:` fields. Keep the deployment
# entry empty instead of placing per-request limits here. Exploration limits
# belong to each robonix/skill/explore/explore request:
#
# - area_hint: string describing the requested exploration area.
# - timeout_s: unsigned integer execution deadline in seconds.
# - max_speed_m_s: floating-point linear speed limit in metres per second.
#
# This file is documentation only and is not loaded by the provider.

config: {}
