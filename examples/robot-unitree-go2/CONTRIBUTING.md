# Contributing

Keep changes small, reviewable and fail-closed. Before opening a pull request:

1. Run `./scripts/validate_offline.sh`.
2. Confirm no test or helper publishes `/cmd_vel`, `/lowcmd` or
   `/api/sport/request`, invokes a posture API, or starts a Unitree motion
   example.
3. Add an offline regression test for every safety, lifecycle or contract
   change.
4. Update `NOTICE` and `THIRD_PARTY.md` when redistributing another component.
5. Do not commit `.env`, logs, maps, bags, generated builds, credentials,
   calibration tied to a private device, or device handover documents.

Hardware evidence must start with the read-only checklist. A pull request
must state explicitly whether it was validated only offline, with recorded
data, or on supervised physical hardware; never imply hardware validation from
unit tests.

## Commit identity and AI assistance

The human contributor remains the sole accountable author and committer. Check
the identity before committing:

```bash
git config user.name
git config user.email
```

Do not put an AI agent or automation account in the Git author or committer
fields. Do not identify AI as an author, DCO signer, reviewer, tester, or
approver through `Co-authored-by`, `Co-developed-by`, `Signed-off-by`,
`Reviewed-by`, `Tested-by`, `Acked-by`, `Suggested-by`, or similar trailers.
Never fabricate a human trailer or copy one from another commit.

When AI materially assists the implementation, the human contributor should
disclose the tool and exact model without an email address:

```text
Assisted-by: AGENT_NAME:MODEL_VERSION [TOOL ...]
```

For a contribution to the Robonix main repository, also follow the current
[official Robonix code contribution guide](https://robonix.syswonder.org/contributing/robonix)
and run its repository-owned check before pushing:

```bash
python3 scripts/check_commit_authorship.py --base origin/dev --head HEAD
```

Documentation changes follow the separate
[Robonix documentation contribution guide](https://robonix.syswonder.org/contributing/documentation),
including its validation and AI-assistance disclosure requirements.
