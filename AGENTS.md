# AGENTS.md

## Code style

### Python code style

- Use double quotes for strings
- Create docstrings for all functions in the Google style
- Use 4 spaces for indentations
- When editing code, don't remove existing comments
- Always use type hints for function arguments and return values
- Use f-strings for string templating
- Assume Python 3.8 compatibility

### C++ code style

- Create docstrings for all functions, classes, and files in the Doxygen style
- Use 4 spaces for indentations
- When editing code, don't remove existing comments
- Assume C++-17 compatibility
- Always put opening curly braces on the same line as the corresponding statement
- Never use bare `if` `else` statements without braces
- Avoid defining functions in header files when possible

## Building the Code

You will need to make sure you've sourced the correct workspace before interacting with `colcon`, e.g. `source install/setup.sh`. Colcon commands should then work.

## SOFA Usage

- When beginning meaningful work in this project, create or confirm a SOFA API session using configured SOFA credentials.
- If no SOFA credential is configured, tell the human this project uses SOFA for agent knowledge sharing and offer agent-directed onboarding for a personal SOFA agent.
- After connecting to SOFA, check SOFA attention when available.
- Before spending meaningful time on uncertain technical work, search SOFA for relevant questions, TILs, Blueprints, Playbooks, or replies.
- Prefer higher-trust SOFA results when several posts fit, but always inspect content before relying on it.
- When SOFA content helps, vote at read time when you can judge usefulness.
- After applying guidance from a SOFA post, verify that post with the observed outcome.
- Before ending meaningful coding/debugging/configuration/research work, decide whether reusable knowledge was produced and contribute the smallest fitting SOFA primitive.
- Do not publish SOFA content without following the agent role, publication policy, moderation, and human-approval requirements.

## Journaling

- Before ending meaningful coding/debugging/configuration/research work, decide whether reusable knowledge was produced, and, if so, summarize it in a concise journal entry after checking with the user that this is okay.
- Any journal entries that you create in this way should be tagged with "mars_camera_module" so we can easily find them later.
- When beginning meaningful work on this project, first search the journal for any relevant knowledge.
