# Contributing Guidelines

Thank you for your interest in contributing to the **AI-Based Autonomous Navigation System**.

## 1. Code Standards and Architecture Guidelines

- **Package Architecture**: All reusable core logic must reside under `src/autonomous_nav/`. Entry scripts reside under `scripts/`.
- **Typing**: Use Python type annotations (`typing`) across all function and class signatures.
- **Documentation**: Provide clear, NumPy- or Google-style docstrings describing parameters, return values, and mathematical formulas or assumptions.
- **Dependencies**: Keep core dependencies minimal (`numpy`, `matplotlib`, `scipy`). Optional extensions (`opencv-python`, `pygame`) must be gracefully imported or isolated.
- **Pure Functions & Dataclasses**: Prefer immutable or state-explicit dataclasses for configurations, states, and coordinates.

## 2. Testing Policy

- Every new module or algorithm must be accompanied by unit tests in `tests/unit/`.
- Scenario or end-to-end changes must validate against `tests/scenarios/` and `tests/integration/`.
- Run the full test suite locally before submitting changes:
  ```powershell
  pytest tests/ -v
  ```

## 3. Pull Request Process

1. Create a feature branch (`git checkout -b feature/my-feature`).
2. Ensure all tests pass with zero regressions.
3. Update relevant documentation in `docs/` and add an entry in `docs/execution_log.md`.
4. Open a Pull Request with a concise summary of implemented changes and verification evidence.

## 4. License Notice

Licensing terms are maintained by the project owner. Contributions are submitted under the project's prospective open-source terms.

