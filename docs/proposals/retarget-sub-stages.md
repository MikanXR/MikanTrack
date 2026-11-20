# Proposal: sub-stages for the Retarget stage

Accepted and built 2026-11-20 on the `direct-mode-ui-regroup` branch. The Decisions below are the answers from one round of questions. Two box values were shortened in the build to fit the box's value slot: `bad bones: N` for the mapping problem count and `on shoulders` for the followed placement.

The Retarget stage is one box whose Inspector holds everything about the avatar: the model, its bone map, its placement and the chains. This splits it into a lane of three sub-stages, the way Capture, Direct solver, Output and Face already are, so each concern has its own box, status and Inspector page.

## Findings

- A sub-stage is an `ePipelineStage` entry whose `k_stages` row names its parent, a lane in `PipelineModel::getSubStageLanes`, a status in `computeStatus`, a name and description in the string table, and its cases in `StageContent::hasSection` and `drawSection`. The Inspector, the pinned windows and the `pipeline` automation namespace pick a new one up from those.

- The Retarget stage today (`StageContent.cpp:358-375`, `:401-421`):
	- Inputs and calibration: `AvatarPanel::drawLoadControls`, and with a model `drawModelSummary`
	- Settings: with a model, Placement and Mapping as closed tree nodes and the Preview pose radio, then `drawChains` in every case
	- Diagnostics: `SettingsPanels::drawRetargetChainReadout`

- `AvatarPanel` already draws each section as its own function, each drawable twice in one frame, with the preview pose, the mapping selection and the chain selection as panel state. Nothing has to move out of the panel.

- Without a model the chains still show the two arms' loss controls, because the stream applies those rules to the measured-length skeleton.

- Placement in a direct project: Follow shoulders and the fixed root are read only by the position retarget (`AppConfig.cpp:397-399`, `AvatarRetarget.cpp:194-236`). The direct solver reads neither. Show in scene (`MainWindow.cpp:649`) is the only placement field with an effect in a direct project.

- The Retarget status has three outcomes (`PipelineModel.cpp:389-396`): not configured without an avatar, inactive with nothing to pose, running with the count of placed arms. A mapping problem (a required bone unmapped, one node on two bones, the swapped-finger warning) shows only as a dot in the Mapping figure, computed by the private `AvatarPanel::computeDotStates`.

- A click on a source bone in the Retarget view sets the chain selection (`MainWindow.cpp:680-682`) and leaves the pipeline selection alone.

- The Stage View's scene follows the top-level stage, so a Retarget sub-stage shows the Retarget view with no change.

## Decisions

- The lane is VRM model, Body placement, IK chains, in that order, labeled in the sentence case the other boxes use.
- The top-level Retarget keeps its status, the Preview pose radio and the per-chain state readout. The model, placement and chain settings leave it.
- The Preview pose radio is drawn on Retarget and on all three sub-stages, over the one shared state.
- VRM model: Inputs and calibration holds the load controls and the model summary. Settings holds Mapping drawn open, with no tree node.
- Body placement is hidden in a direct project. Show in scene moves to VRM model so both modes reach it.
- Without an avatar the lane keeps all its boxes: VRM model not configured, Body placement inactive with no settings, IK chains running with the two arms' loss controls.
- Each box reports its own health: VRM model the mapped bone count or a mapping problem, Body placement whether the shown frame follows the shoulders or stands at the fixed root, IK chains the placed arm count.
- A mapping problem on VRM model turns the top-level Retarget degraded too, with its value unchanged.
- A chain picked in the Retarget view moves the pipeline selection to IK chains only when the top-level Retarget is selected.
- Only IK chains has a Diagnostics section, the per-chain state readout.

## Design

### The stages and the lane

Three entries join `ePipelineStage` after `Retarget`, with `S::Retarget` as their parent:

- `VrmModel`, token `model`
- `Placement`, token `placement`
- `Chains`, token `chains`

`getSubStageLanes` gains a `S::Retarget` case. It needs the pipeline shape, which it does not take today, so it gains a `bDirect` parameter that its callers pass from the inputs they already hold. A multi-camera project's lane is `{VrmModel, Placement, Chains}` and a direct project's is `{VrmModel, Chains}`. The lane follows the shown project's shape, like every other lane, so a direct take replayed in a multi-camera project shows two boxes.

### Status

`PipelineStatusInputs` gains two fields, filled in `MainWindow::gatherPipelineInputs` beside `bAvatarLoaded`:

- `avatarMappedBoneCount`: the humanoid bones with a node
- `avatarMappingProblemCount`: the bones whose dot state is a warning or a required bone unmapped

The dot states move from `AvatarPanel::computeDotStates` to a free function beside `AvatarMapFigure`, so the panel and the inputs share one rule.

The rules:

- VRM model
	- Not configured without an avatar, with Retarget's value and reasons (`valNoAvatar`, `whyRetargetNoAvatar` or `whyDirectNoAvatar`)
	- Degraded with `mapping problems: N` and a reason that points at the Mapping section when the problem count is above zero
	- Running with `bones: N` otherwise
- Body placement
	- Inactive without an avatar, with the reason that there is no avatar to place
	- Inactive with nothing to pose, as Retarget is
	- Running with `following shoulders` when Follow shoulders is on and the shown frame has both shoulders measured, the condition `AvatarRetarget.cpp:194` tests, and `fixed root` otherwise
- IK chains
	- Running with `loss rules only` without an avatar
	- With an avatar, Retarget's rules: inactive with nothing to pose, running with `arms placed: N`
- Retarget
	- Today's three rules, then degraded with the mapping reason and the `arms placed: N` value when it would be running and the problem count is above zero

### Inspector sections

`StageContent::hasSection` and `drawSection` change as follows. Every Settings section that needs a model starts with the Preview pose radio.

- Retarget
	- Inputs and calibration: none
	- Settings: with a model, the Preview pose radio. No section without one.
	- Diagnostics: the per-chain state readout
- VRM model
	- Inputs and calibration: the load controls, and with a model the model summary
	- Settings: with a model, the Preview pose radio, Show in scene, then Mapping drawn open
	- Diagnostics: none
- Body placement
	- Settings: with a model, the Preview pose radio, Follow shoulders, and the fixed root with its reset
	- No other section, and no section without a model
- IK chains
	- Settings: with a model the Preview pose radio, then the chain row and the selected chain's settings, the arms' loss controls alone without a model
	- Diagnostics: the per-chain state readout

`AvatarPanel::drawPlacement` loses Show in scene, which becomes `drawShowInScene`. `StageContent::drawRetargetSettings` and the two tree nodes go away, with the `avatarPanel.tabPlacement` and `avatarPanel.tabMapping` strings if nothing else reads them.

### The chain pick

Where `MainWindow` consumes the Retarget view's picked chain it also checks the pipeline selection. If that is the top-level Retarget it selects `retarget.chains`. Any other selection stays. `pipeline chain <id>` sets the chain only, since a drive selects its stage itself.

### Strings

New keys in `en.json` and `ja.json`, the Japanese ones machine-translated and added to the human-review list in `docs/plan.md`:

- the three stage names and their descriptions
- `valBonesFmt`, `valMappingProblemsFmt`, `valFollowingShoulders`, `valFixedRoot`, `valLossRulesOnly`
- `whyMappingProblems`, `whyPlacementNoAvatar`

### Docs

- `docs/reference/architecture.md`: the Pipeline panel paragraph names the Retarget lane
- `docs/reference/avatar.md`: where the panel's sections are hosted, and the status rules for a mapping problem
- `docs/reference/automation.md`: the three stage ids, and that a Retarget view pick can move the selection
- `docs/reference/testing.md`: the new `--test-pipeline-model` cases
- `docs/plan.md`: this item removed, the Japanese strings and the live checks added

## Verification

- `--test-pipeline-model` gains:
	- the Retarget lane in a multi-camera shape (three boxes) and a direct shape (two)
	- each sub-stage's state, value and reason with no avatar, with nothing to pose, with a clean model, and with a mapping problem
	- Body placement's two running values
	- the Retarget roll-up
	- the three ids through the text id parse and format round trip
- Its check that every status text is reachable covers the new strings, so each needs an input that reaches it.
- `--loc-test` for key parity and the format specifiers of the two `Fmt` values.
- The full self-test batch, read from the logs.
- Through `tools/automate.py`, with screenshots:
	- a multi-camera project with an avatar: the three boxes, each Inspector page, Mapping open, the Preview pose radio on all four and one change showing on the others
	- a direct project with an avatar: two boxes, Show in scene on VRM model
	- no avatar in both modes: the states and reasons, and the arms' loss controls on IK chains
	- `pipeline select`, `status` and `pin` on the three ids
	- a click on a source bone with Retarget selected moves the selection to IK chains, and with VRM model selected does not
	- a model with one node on two bones: VRM model and Retarget both degraded
- No replayed stage changes, so `--replay-verify` is not evidence here and no checksum moves.
