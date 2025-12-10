<!--
SYNC IMPACT REPORT (2025-12-09)
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
Version Change: N/A → 1.0.0
Bump Rationale: Initial constitution ratification for Physical AI & Humanoid
                Robotics book + RAG chatbot project

Modified Principles:
  - ALL principles created from template (first ratification)

Added Sections:
  - Core Principles (7 principles)
  - Technical Standards
  - Development Workflow
  - Governance

Removed Sections:
  - None (initial creation)

Templates Status:
  ✅ plan-template.md — Constitution Check section aligns with principles
  ✅ spec-template.md — Requirements structure supports reproducible code & testing
  ✅ tasks-template.md — Task categorization supports modular implementation
  ⚠️  MANUAL REVIEW: Ensure commands reference robotics-specific requirements

Follow-up TODOs:
  - Monitor compliance as first features developed
  - Refine citation/source verification workflow if automation needed
  - Establish peer review process for technical accuracy validation
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
-->

# Physical AI & Humanoid Robotics Book + RAG Chatbot Constitution

## Core Principles

### I. Technical Accuracy & Source Verification

ALL technical content, code examples, robotics configurations, and architectural
guidance MUST be verifiable against authoritative sources. No hallucinations permitted.

**Mandates:**
- Every robotics framework claim (ROS 2, Gazebo, Isaac Sim, Unity) MUST cite
  official documentation, research papers, or verified community resources
- All code snippets MUST be tested and runnable in specified environments
- Simulation configurations (URDF, SDF, Isaac worlds) MUST reference working examples
- Vision-Language-Action (VLA) model claims MUST link to published research
- Minimum 30 IEEE-style citations across book content
- Citation format: `[AuthorYear]` inline, full reference in bibliography

**Rationale:** Reader trust depends on precision. Robotics engineers require
reproducible, verifiable workflows. Inaccurate ROS 2 APIs or Isaac SDK usage
destroys credibility and wastes implementation time.

### II. Reproducible Code & Simulations

Every code example, ROS 2 node, simulation setup, and deployment pipeline MUST
be executable by readers with documented prerequisites.

**Mandates:**
- ROS 2 rclpy nodes include complete imports, dependencies (package.xml), build
  instructions (CMakeLists.txt or setup.py)
- Gazebo/Unity digital twin scenes include asset manifests, launch files, and
  environment setup steps
- NVIDIA Isaac Sim examples specify Isaac version, extensions, and reproduction steps
- FastAPI RAG backend includes requirements.txt, environment variables (.env template),
  database schema (Neon Postgres), vector store config (Qdrant)
- Each module's code tested in isolated environment before inclusion
- Capstone project includes end-to-end pipeline script (Whisper ASR → planner →
  Nav2 navigation → vision → manipulation)

**Rationale:** "It works on my machine" is unacceptable. Robotics development
has complex dependencies; explicit reproduction steps prevent reader frustration
and enable learning by doing.

### III. Modular Structure with Clear Scope

Book content and RAG system MUST follow defined modular boundaries. Each module
independently testable and documented.

**Mandates:**
- Four core modules with explicit scope:
  1. ROS 2 Control System (publishers, subscribers, services, actions, parameters)
  2. Digital Twin (Gazebo simulation + Unity visualization, sensor simulation)
  3. Isaac Perception & Navigation (camera processing, Nav2 integration, obstacle avoidance)
  4. Vision-Language-Action Robotics (VLA model integration, multimodal planning)
- Capstone synthesizes modules into unified humanoid robot pipeline
- RAG chatbot scope: answer ONLY from book text + curated robotics content
  (no general knowledge outside domain)
- Each module has dedicated spec/plan/tasks artifacts in specs/ directory
- Cross-module dependencies explicitly documented in plan.md files

**Rationale:** Modular design enables parallel development, isolated testing,
and incremental reader learning. Clear scope prevents RAG chatbot from providing
incorrect out-of-domain answers.

### IV. Engineering-Focused Writing (NON-NEGOTIABLE)

Writing style MUST prioritize clarity, precision, and actionability for software/
robotics engineers. No marketing fluff, no hand-waving.

**Mandates:**
- Explain concepts with concrete examples: "Nav2 uses Dijkstra's algorithm for
  global planning" not "Nav2 intelligently finds paths"
- Include failure modes: "If tf tree missing /map → /base_link, Nav2 throws
  TransformException"
- Provide debugging guidance: "Use `ros2 topic echo /cmd_vel` to verify velocity
  commands"
- Avoid vague language: replace "should" with "MUST" or "SHOULD (with rationale)"
- Code comments explain WHY, not WHAT: `# Quaternion required for ROS 2 Pose msg`
  not `# Create quaternion`
- Architectural decisions justified: "FastAPI chosen over Flask for async support
  with RAG inference pipeline"

**Rationale:** Engineers need actionable information, not inspiration. Precision
reduces ambiguity and accelerates implementation. Vague content wastes reader time.

### V. Citation & Documentation Standards

All external claims, research references, and tool documentation MUST follow
IEEE citation style and maintain verifiable links.

**Mandates:**
- IEEE format: `[1] A. Author, "Title," Conference/Journal, vol. X, no. Y, pp. Z, Year.`
- Inline citations: `ROS 2 uses DDS for middleware [ROS2Docs2023]`
- Bibliography section at book end with all references
- Minimum 30 authoritative sources across book (official docs, research papers,
  verified tutorials)
- Broken links checked before publication (automated link validation)
- Version pinning: "ROS 2 Humble (2022 LTS release)" not "latest ROS 2"
- Official docs prioritized: ros.org, docs.nvidia.com/isaac, gazebosim.org

**Rationale:** IEEE standard for technical publications. Readers must trace claims
to sources. Version pinning prevents compatibility issues. Link validation ensures
long-term reference integrity.

### VI. Testability & Validation

Every component (book code, RAG backend, simulations) MUST have defined validation
criteria and acceptance tests.

**Mandates:**
- ROS 2 code: unit tests (pytest), integration tests (launch_testing), behavior
  validation (e.g., robot reaches goal pose)
- Simulations: visual inspection criteria (Gazebo GUI screenshots), sensor output
  validation (e.g., /scan topic publishes LaserScan)
- RAG chatbot: contract tests (API schema), response accuracy tests (ground truth Q&A),
  scope validation (rejects out-of-domain queries)
- Docusaurus build: CI/CD pipeline verifies `npm run build` succeeds, no broken links
- Capstone pipeline: end-to-end test (voice command → robot action, captured in video/logs)
- Acceptance criteria in spec.md for each module (measurable outcomes)

**Rationale:** Untested code/content breeds errors. Validation criteria enable
quality gates. Readers trust tested examples; contributors can verify changes
don't break existing functionality.

### VII. Deployment & Accessibility

Book and RAG chatbot MUST be publicly accessible with minimal setup friction.

**Mandates:**
- Book hosted on GitHub Pages (static Docusaurus site)
- Deployment automated via GitHub Actions (build on push to main)
- RAG chatbot embedded in book interface (iframe or React component)
- FastAPI backend deployed to verifiable hosting (Render, Railway, or documented
  self-hosting with Docker)
- Database (Neon Postgres) and vector store (Qdrant Cloud/self-hosted) connection
  strings documented
- Reader setup: clear prerequisites (Node.js version, Python version, ROS 2 distro,
  Isaac Sim version)
- Mobile-responsive book design (Docusaurus default theme acceptable)

**Rationale:** Inaccessible content has zero impact. GitHub Pages provides free,
reliable hosting. Automated deployment prevents manual errors. Embedded RAG chatbot
provides immediate value to readers exploring content.

## Technical Standards

### Technology Stack Requirements

**Book Platform:**
- Docusaurus (latest stable): static site generation, built-in search, React-based
  theming
- Deployment: GitHub Pages via GitHub Actions
- Minimum Node.js: v18.x
- Content format: MDX (Markdown + JSX for interactive components)

**Robotics Stack:**
- ROS 2 Humble Hawksbill (LTS, Ubuntu 22.04)
- Gazebo Fortress (or Gazebo Classic 11 if legacy required)
- Unity 2022.3 LTS + ROS-TCP-Connector
- NVIDIA Isaac Sim 2023.1.1 (or latest stable)
- Python 3.10+ for rclpy nodes

**RAG Chatbot Stack:**
- Backend: FastAPI (async support for LLM inference)
- LLM: OpenAI GPT-4o via OpenAI Agents API / ChatKit
- Database: Neon Postgres (serverless PostgreSQL)
- Vector Store: Qdrant (for semantic search over book embeddings)
- Embeddings: OpenAI text-embedding-3-small (or open-source alternative)
- Frontend Integration: React component or iframe within Docusaurus

### Performance & Constraints

**Book:**
- Target word count: 25,000–40,000 words
- Build time: < 2 minutes on GitHub Actions
- Page load: < 3s on 3G connection (Docusaurus optimization)

**RAG Chatbot:**
- Response latency: < 5s for 95th percentile queries
- Scope enforcement: reject queries outside book/robotics domain with polite message
- Embedding refresh: automated pipeline to re-index on book content updates

**Simulations:**
- Gazebo real-time factor: ≥ 0.8 for humanoid robot simulation (on recommended hardware)
- Isaac Sim: documented minimum GPU requirements (RTX 3060 or equivalent)

### Security & Compliance

**Secrets Management:**
- OpenAI API keys, database credentials in environment variables (never committed)
- .env.example template provided (with placeholder values)
- GitHub Secrets for CI/CD deployment keys

**Data Handling:**
- RAG chatbot does NOT store user queries (stateless API)
- Optional analytics (privacy-respecting, documented, opt-in)

**Code Safety:**
- No arbitrary code execution in RAG responses (output sanitized)
- ROS 2 nodes follow security best practices (parameter validation, no shell injection)

## Development Workflow

### Spec-Driven Development (SDD) Enforcement

ALL features (book modules, RAG components, simulations) MUST follow SDD workflow
via Spec-Kit Plus templates:

1. **Specification (`/sp.specify`)**: User requirements → spec.md
   - User stories with independent testing criteria
   - Functional requirements (FR-XXX)
   - Success criteria (SC-XXX)

2. **Planning (`/sp.plan`)**: Architecture → plan.md + artifacts
   - Technical context (ROS 2 packages, Isaac extensions, FastAPI routes)
   - Constitution check (validate against this file)
   - Research, data models, contracts, quickstart

3. **Task Breakdown (`/sp.tasks`)**: Implementation checklist → tasks.md
   - Dependency-ordered tasks
   - Parallelization opportunities marked [P]
   - Module/story labels ([US1], [US2])

4. **Implementation (`/sp.implement`)**: Execute tasks → working code
   - TDD optional (tests written before code if requested)
   - Commit after logical task groups
   - Validate against acceptance criteria

5. **Review & Documentation**: ADRs, PHRs, commit/PR
   - Architectural decisions → `/sp.adr` (on user consent)
   - Prompt history → `/sp.phr` (automatic after user prompts)
   - Pull requests → `/sp.git.commit_pr` (conventional commits)

### Quality Gates

**Pre-Commit:**
- Code formatting (Black for Python, Prettier for JS/MDX)
- ROS 2 package.xml/CMakeLists.txt validity (for ROS packages)
- No secrets in code (git-secrets or pre-commit hook)

**Pre-Merge:**
- Spec acceptance criteria met (checklist in PR description)
- Build succeeds (Docusaurus, ROS 2 colcon build, FastAPI tests)
- Constitution compliance verified (no violations without justification in plan.md)

**Pre-Deployment:**
- End-to-end tests pass (book builds, RAG chatbot responds correctly, simulations launch)
- No broken links (automated check)
- Capstone pipeline demonstrated (video or recorded terminal session)

### Review Process

**Content Reviews (Book Modules):**
- Technical accuracy: verify code runs, citations valid
- Clarity: engineer peer review (can they follow instructions?)
- Completeness: module covers stated scope in spec.md

**Code Reviews (RAG/ROS 2):**
- Functionality: meets spec requirements
- Reproducibility: reviewer can run locally
- Documentation: README/comments explain setup

**ADR Reviews (Architectural Decisions):**
- Justification: tradeoffs documented
- Alternatives: at least 2 options considered
- Reversibility: can decision be changed later? (mark irreversible decisions)

## Governance

### Amendment Procedure

1. Propose amendment (PR to `.specify/memory/constitution.md`)
2. Justify change (what problem does amendment solve?)
3. Version bump (MAJOR/MINOR/PATCH semantic versioning)
4. Update Sync Impact Report (comment at top of constitution file)
5. Propagate to templates (update plan/spec/tasks templates if needed)
6. Merge after validation (ensure no broken references)

### Versioning Policy

- **MAJOR (X.0.0)**: Backward-incompatible principle removals or redefinitions
  - Example: Remove "No Unity allowed" → adopt Unity (requires project refactor)
- **MINOR (X.Y.0)**: New principles/sections or material expansions
  - Example: Add "Observability Principle" → requires new logging tasks
- **PATCH (X.Y.Z)**: Clarifications, typo fixes, non-semantic refinements
  - Example: Reword principle for clarity (no behavioral change)

### Compliance Review

- Every `/sp.plan` execution MUST include Constitution Check section
- Violations allowed ONLY with documented justification in plan.md Complexity Tracking table
- Unjustified violations block PR merge
- Periodic audit (quarterly or per major milestone): verify all features comply

### Complexity Justification

If a feature violates a principle (e.g., adds 5th module when constitution specifies 4):

1. Document in plan.md Complexity Tracking table
2. Explain why needed (specific requirement)
3. Explain why simpler alternative rejected
4. Propose constitution amendment if violation should become new standard

**Example Justification:**

| Violation | Why Needed | Simpler Alternative Rejected Because |
|-----------|------------|-------------------------------------|
| 5th module (Hardware Integration) | Physical robot deployment required | Simulation-only insufficient for capstone real-world demo |

### Living Document

This constitution supersedes all other practices when conflicts arise. It is a
living document: as project evolves, amendments expected. Transparency and
documentation (PHRs, ADRs) maintain institutional knowledge.

For runtime development guidance specific to agent workflows, see `CLAUDE.md`.

**Version**: 1.0.0 | **Ratified**: 2025-12-09 | **Last Amended**: 2025-12-09
