# Security and safety reporting

The `main` branch is the supported development line. This repository controls
a physical robot only after several explicit local gates, so treat a bypass of
motion, network, IPC, map-identity, localization or obstacle-staleness checks
as both a security and a safety issue.

Do not publish exploit details, credentials, device serials, private network
data or unredacted robot logs in a public issue. Prefer GitHub's private
security-advisory reporting for this repository, or contact the listed
maintainer through an already trusted private channel. Never send a password,
API key, SSH key or one-time login code.

The deployment pins every non-DDS control, capability, audio and dashboard
listener to loopback and disables the Scene and Mapping administration UIs.
Use authenticated SSH tunnels for remote access. Treat any route that widens a
listener or bypasses the upstream bind-host compatibility gate as a security
and robot-safety defect.

Reports should include the affected commit, hardware/firmware edition, a
minimal offline or bag-replay reproduction and redacted diagnostics. Do not
reproduce a suspected motion-control flaw on a standing or freely moving Go2.
Use the fake-client C++ tests, an isolated ROS domain and a no-op velocity sink
until the issue has been understood and reviewed.
