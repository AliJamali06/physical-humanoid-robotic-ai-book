# Implementation Plan: Module 1 – ROS 2 Robotic Nervous System

**Branch**: `001-ros2-robotic-nervous-system` | **Date**: 2025-12-09 | **Spec**: [spec.md](./spec.md)
**Input**: Feature specification from `/specs/001-ros2-robotic-nervous-system/spec.md`

## Summary

Create Module 1 of the Physical AI & Humanoid Robotics book covering ROS 2 fundamentals for beginner/intermediate students. Module consists of 3 chapters delivered as MDX files in Docusaurus documentation site: (1) ROS 2 Nodes and Topic Communication, (2) Services for Request-Response Patterns, (3) Humanoid URDF Basics. Each chapter includes runnable Python code examples, terminal commands, diagrams, exercises, and IEEE-format citations. Technical approach: Write chapters as standalone MDX files with embedded code blocks, organize under `/docs/module-1-ros2/` directory, configure sidebar navigation, validate code examples in isolated ROS 2 Humble environment, and verify Docusaurus build before integration.

## Technical Context

**Language/Version**: MDX (Markdown + JSX) for content; Python 3.10+ for code examples (ROS 2 Humble rclpy)
**Primary Dependencies**: Docusaurus 4.x (static site generator), ROS 2 Humble (code validation), Prism syntax highlighting (built-in)
**Storage**: Static MDX files in `Humain-robotic-book/docs/module-1-ros2/` directory; code examples in fenced code blocks
**Testing**: Docusaurus build validation (`npm run build`), ROS 2 code execution tests (Python scripts in test workspace), link checker, IEEE citation format validation
**Target Platform**: GitHub Pages (static hosting), readers with ROS 2 Humble on Ubuntu 22.04
**Project Type**: Documentation/educational content (Docusaurus site)
**Performance Goals**: Docusaurus build time <2 minutes, page load <3s on 3G (per constitution), code examples execute <5s
**Constraints**: ROS 2 Humble compatibility required; all code must run on fresh Ubuntu 22.04 install; min 5 IEEE citations per chapter; PEP 8 compliance
**Scale/Scope**: 3 chapters, ~25-30 pages total, 12-15 runnable code examples, 4-6 hour student completion time

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

### Alignment with Constitution Principles

**I. Technical Accuracy & Source Verification** ✅
- Plan requires minimum 5 IEEE citations per chapter (FR-031) citing official ROS 2 Humble docs
- Code examples must be tested on ROS 2 Humble before inclusion (FR-026, SC-006)
- URDF examples validated with `check_urdf` command (FR-022)
- Quality checks include technical accuracy validation against docs.ros.org

**II. Reproducible Code & Simulations** ✅
- All code examples include complete imports, no omissions (FR-005)
- Package structure documented with `package.xml`, `setup.py` (FR-004)
- Prerequisites section lists exact ROS 2 packages and dependencies (FR-027)
- Testing strategy includes execution in isolated environment

**III. Modular Structure with Clear Scope** ✅
- Module 1 is first of 4 planned modules (aligns with constitution's 4-module structure)
- Explicitly scoped: nodes, topics, services, URDF only - excludes Nav2/perception (spec input)
- Each chapter independently testable (user stories P1, P2, P3)
- Clear boundaries: no advanced topics like Nav2, full simulation, or deployment

**IV. Engineering-Focused Writing** ✅
- Concrete examples required: publisher at 1Hz, QoS RELIABLE vs BEST_EFFORT (FR-002, FR-006)
- Failure modes documented: "Common Errors" section mandatory (FR-030)
- Debugging guidance: `ros2 topic echo`, `ros2 node list` commands (FR-008)
- No vague language: requirements use MUST, not "should" or "can"

**V. Citation & Documentation Standards** ✅
- IEEE format mandated: `[ROS2Docs2023]` inline citations (FR-031)
- Version pinning: ROS 2 Humble explicitly specified (FR-026)
- Official docs prioritized: docs.ros.org, design.ros2.org
- Testing includes automated citation format validation

**VI. Testability & Validation** ✅
- Acceptance criteria in spec for each user story (20 min, 15 min, 10 min task completion)
- Code validation: CI test pipeline for all examples (SC-006)
- Docusaurus build: `npm run build` must succeed (quality gate)
- Student exercises with self-assessment criteria (FR-009, FR-016, FR-024)

**VII. Deployment & Accessibility** ✅
- Docusaurus hosted on GitHub Pages (constitution requirement)
- Mobile-responsive (Docusaurus default theme)
- Setup prerequisites clearly documented (ROS 2 Humble, Ubuntu 22.04)
- Build automated via npm scripts (constitution specifies GitHub Actions)

### Gates Summary
- ✅ All 7 constitution principles satisfied
- ✅ No violations requiring justification
- ✅ Module 1 aligns with 4-module architecture (1 of 4)
- ✅ Ready to proceed to Phase 0 research

## Project Structure

### Documentation (this feature)

```text
specs/001-ros2-robotic-nervous-system/
├── plan.md              # This file
├── research.md          # Phase 0: Docusaurus patterns, ROS 2 API research
├── data-model.md        # Phase 1: Chapter structure, code example templates
├── contracts/           # Phase 1: Chapter content contracts
│   ├── chapter-1-nodes-topics.md
│   ├── chapter-2-services.md
│   └── chapter-3-urdf.md
├── quickstart.md        # Phase 1: Development workflow for content authors
├── spec.md              # Requirements (already created)
└── checklists/
    └── requirements.md  # Quality checklist (already created)
```

### Source Code (Docusaurus book content)

```text
Humain-robotic-book/
├── docs/
│   ├── intro.md                              # Existing landing page
│   ├── module-1-ros2/                        # NEW: Module 1 content
│   │   ├── _category_.json                   # Sidebar metadata
│   │   ├── 01-nodes-and-topics.mdx           # Chapter 1
│   │   ├── 02-services.mdx                   # Chapter 2
│   │   ├── 03-urdf-basics.mdx                # Chapter 3
│   │   └── assets/                           # Chapter diagrams/images
│   │       ├── ros2-node-diagram.svg
│   │       ├── topic-pub-sub-flow.svg
│   │       ├── service-request-response.svg
│   │       └── urdf-link-tree-example.svg
│   ├── tutorial-basics/                      # Existing (to be replaced/archived)
│   └── tutorial-extras/                      # Existing (to be replaced/archived)
├── src/
│   ├── components/                           # Future: interactive components
│   └── css/
│       └── custom.css                        # Styling customizations
├── static/
│   └── img/                                  # Logos, favicons
├── docusaurus.config.ts                      # Site config (update title, metadata)
├── sidebars.ts                               # Navigation (auto-generated from structure)
├── package.json                              # Dependencies
└── tsconfig.json                             # TypeScript config

# Code validation workspace (separate from book content)
ros2_code_examples/                           # NEW: Testing workspace
├── src/
│   └── module1_examples/                     # ROS 2 package for validation
│       ├── package.xml
│       ├── setup.py
│       ├── module1_examples/
│       │   ├── __init__.py
│       │   ├── publisher_example.py          # From Chapter 1
│       │   ├── subscriber_example.py         # From Chapter 1
│       │   ├── service_server.py             # From Chapter 2
│       │   ├── service_client.py             # From Chapter 2
│       │   └── urdf/
│       │       └── simple_humanoid.urdf      # From Chapter 3
│       └── test/
│           ├── test_publisher.py             # Validation tests
│           ├── test_subscriber.py
│           ├── test_service.py
│           └── test_urdf_validity.py
└── README.md                                 # Setup instructions
```

**Structure Decision**:
- **Docusaurus book**: Use existing `Humain-robotic-book/` structure; add new `docs/module-1-ros2/` directory for Module 1 content
- **Code validation**: Separate `ros2_code_examples/` workspace for testing code before embedding in MDX (prevents polluting book repo with ROS 2 build artifacts)
- **Sidebar navigation**: Auto-generated from filesystem structure using `_category_.json` metadata (Docusaurus convention)
- **Assets**: Co-locate diagrams with chapters in `module-1-ros2/assets/` for easy reference
- **Rationale**: Separation of concerns (content vs code validation), aligns with Docusaurus best practices, supports incremental chapter delivery

## Complexity Tracking

> **Fill ONLY if Constitution Check has violations that must be justified**

No violations detected. All requirements align with constitution principles.

## Phase 0: Research

**Objective**: Validate technical assumptions and document Docusaurus patterns for educational content

**Research Tasks**:
1. **Docusaurus MDX capabilities**: Confirm support for:
   - Syntax highlighting for Python, Bash, XML (required for FR-032)
   - Custom Prism language definitions (if needed for ROS 2 message/service types)
   - Embedded diagrams (SVG, Mermaid)
   - Admonitions for warnings, tips, exercises
   - Code block features: line numbers, highlighting, file names

2. **ROS 2 Humble API verification**: Document exact API signatures for:
   - `rclpy.node.Node` class (constructor, create_publisher, create_subscription)
   - Topic QoS profiles (default, RELIABLE, BEST_EFFORT)
   - Service patterns (create_service, create_client, wait_for_service)
   - Standard message types: `std_msgs/msg/String`, `example_interfaces/srv/AddTwoInts`
   - URDF validation tools: `check_urdf`, `urdf_to_graphiz` (installation, usage)

3. **Citation format patterns**: Research IEEE citation style for:
   - Online documentation (ROS 2 docs.ros.org)
   - GitHub repositories (rclpy, urdf packages)
   - Software documentation without traditional authors
   - Example: `[ROS2Docs2023] "ROS 2 Documentation: Humble," Open Robotics, docs.ros.org/en/humble/, 2023.`

4. **Existing Docusaurus educational sites**: Study patterns from:
   - ROS 2 official documentation (docs.ros.org structure)
   - Technical tutorial sites (code examples, exercises, navigation)
   - Identify: How do they handle runnable code? Exercise formats? Prerequisite sections?

**Deliverable**: `research.md` with findings, code snippets, and decision justifications

## Phase 1: Design

**Objective**: Define chapter structure, content contracts, and code example templates

### Data Model (Chapter Structure)

**Deliverable**: `data-model.md` defining:
- **Chapter template schema**:
  - Frontmatter (title, description, keywords, sidebar position)
  - Section order: Prerequisites → Learning Objectives → Concept Explanation → Code Examples → Exercises → Common Errors → References
  - Metadata fields for tracking (word count, estimated time, difficulty)

- **Code example template**:
  - Structure: Explanation → Full code block → Execution command → Expected output → Explanation of key lines
  - Boilerplate: Package imports, `main()` function, `rclpy.init()`, `rclpy.shutdown()`
  - Annotations: Inline comments (WHY, not WHAT per constitution)
  - Validation checklist: Runs on fresh ROS 2 Humble install, PEP 8 compliant, includes docstrings

- **Exercise template**:
  - Format: Objective → Instructions (numbered steps) → Verification method → Solution (collapsed/spoiler)
  - Difficulty levels: Basic (modify variable), Intermediate (add feature), Advanced (combine concepts)

- **Diagram conventions**:
  - Tool: SVG (vector graphics for scaling, accessibility)
  - Style: Consistent color scheme, node shapes (rectangles for processes, ovals for data)
  - Labels: Font size, arrow directions (publisher → topic → subscriber)

### Contracts (Chapter Content Specifications)

**Deliverable**: `contracts/` directory with 3 files:

**`chapter-1-nodes-topics.md`** (maps to FR-001 through FR-009):
- **Word count target**: 2500-3000 words
- **Code examples**: 4 total
  1. Minimal publisher (string messages, 1Hz)
  2. Minimal subscriber (print to terminal)
  3. Publisher with custom message content
  4. QoS profile demonstration (RELIABLE vs BEST_EFFORT)
- **Diagrams**: 2 required
  1. ROS 2 node architecture (node → executor → callback)
  2. Topic publish-subscribe flow (publisher → topic → subscriber with message type)
- **Exercises**: 3 exercises
  1. Modify message content in publisher
  2. Change topic name and verify subscriber receives
  3. Experiment with QoS settings (observe behavior differences)
- **Prerequisites list**: ROS 2 Humble, Python 3.10+, colcon, rclpy package
- **Learning objectives**: 5 measurable outcomes (e.g., "Create publisher node in <10 minutes")
- **Common errors**: 5 examples (forgot `rclpy.init()`, wrong topic name, missing import)
- **Citations**: Minimum 5 (docs.ros.org/humble, rclpy GitHub, DDS documentation)

**`chapter-2-services.md`** (maps to FR-010 through FR-016):
- **Word count target**: 2000-2500 words
- **Code examples**: 4 total
  1. Service server (AddTwoInts from example_interfaces)
  2. Service client (synchronous call)
  3. Service with timeout handling (`wait_for_service`)
  4. Custom service logic (e.g., unit conversion)
- **Diagrams**: 1 required
  1. Service request-response sequence (client → request → server → response → client)
- **Exercises**: 3 exercises
  1. Modify service computation logic
  2. Create client that handles timeout
  3. Implement custom service (e.g., string reversal)
- **Prerequisites list**: Completion of Chapter 1, example_interfaces package
- **Learning objectives**: 4 measurable outcomes
- **Common errors**: 4 examples (server not started, client timeout, wrong service type)
- **Citations**: Minimum 5

**`chapter-3-urdf.md`** (maps to FR-017 through FR-025):
- **Word count target**: 2500-3000 words
- **Code examples**: 3 total (XML, not Python)
  1. Annotated simple humanoid URDF (4 links, 3 joints)
  2. Joint limit modifications
  3. Adding a new link (e.g., hand)
- **Diagrams**: 2 required
  1. Link coordinate frames (visual representation)
  2. Link-joint tree (from `urdf_to_graphiz` output)
- **Exercises**: 3 exercises
  1. Identify joint types in provided URDF
  2. Modify joint limits and validate
  3. Add simple link (e.g., camera or hand)
- **Prerequisites list**: ROS 2 Humble, urdfdom package, Graphviz (for `urdf_to_graphiz`)
- **Learning objectives**: 4 measurable outcomes
- **Common errors**: 5 examples (missing closing tag, circular dependencies, invalid joint type)
- **Citations**: Minimum 5 (URDF spec, urdf_parser docs, ROS 2 URDF tutorials)

### Quickstart (Development Workflow)

**Deliverable**: `quickstart.md` documenting:
- **Setup**: Clone repo, install dependencies (`npm install`, ROS 2 Humble sourcing)
- **Writing workflow**:
  1. Create branch for chapter (e.g., `feature/module1-chapter1`)
  2. Write MDX file in `docs/module-1-ros2/`
  3. Add frontmatter, follow chapter contract
  4. Embed code examples (validate in `ros2_code_examples/` first)
  5. Create diagrams, save in `assets/`
  6. Run local preview: `npm run start`
  7. Validate build: `npm run build`
  8. Run link checker, citation formatter
  9. Commit, create PR with checklist
- **Testing workflow**:
  1. Code validation: Create ROS 2 package in `ros2_code_examples/`
  2. Test execution: Run each code example, verify output
  3. Edge cases: Test error scenarios from "Common Errors" section
  4. Copy validated code to MDX (no modifications)
- **Review checklist**:
  - [ ] Chapter meets word count target
  - [ ] All code examples tested and runnable
  - [ ] Minimum citations met (5 per chapter)
  - [ ] Diagrams render correctly
  - [ ] Exercises have verification methods
  - [ ] Prerequisites complete and accurate
  - [ ] Learning objectives measurable
  - [ ] Common Errors section includes 4-5 examples
  - [ ] Docusaurus build succeeds
  - [ ] Links valid (no 404s)

## Phase 2: Implementation Phases

**Note**: Detailed tasks generated by `/sp.tasks` command (not in this plan)

**High-level implementation flow**:

### Phase 2.1: Structure Setup
- Configure Docusaurus for Module 1 (update `docusaurus.config.ts`, `sidebars.ts`)
- Create `docs/module-1-ros2/` directory with `_category_.json`
- Set up ROS 2 code validation workspace (`ros2_code_examples/`)
- Create diagram asset directory structure

### Phase 2.2: Chapter 1 - Nodes and Topics (Priority: P1)
- Write chapter content following `chapter-1-nodes-topics.md` contract
- Develop and test 4 code examples in validation workspace
- Create 2 diagrams (node architecture, pub-sub flow)
- Write 3 exercises with verification methods
- Add 5+ IEEE citations
- Write "Common Errors" section
- Validate Docusaurus build

### Phase 2.3: Chapter 2 - Services (Priority: P2)
- Write chapter content following `chapter-2-services.md` contract
- Develop and test 4 code examples
- Create 1 diagram (service sequence)
- Write 3 exercises
- Add 5+ IEEE citations
- Write "Common Errors" section
- Validate Docusaurus build

### Phase 2.4: Chapter 3 - URDF (Priority: P3)
- Write chapter content following `chapter-3-urdf.md` contract
- Create and validate 3 URDF examples
- Generate 2 diagrams (coordinate frames, link tree from `urdf_to_graphiz`)
- Write 3 exercises
- Add 5+ IEEE citations
- Write "Common Errors" section
- Validate Docusaurus build

### Phase 2.5: Integration and Review
- Test cross-chapter navigation
- Verify all internal links
- Run full Docusaurus build (`npm run build`)
- Validate against success criteria (SC-001 through SC-010)
- Create module landing page (overview, prerequisites, expected time)
- Update main site navigation
- Final constitution compliance check

## Testing Strategy

### Build Validation (Continuous)
**Tools**: npm scripts, Docusaurus CLI
- `npm run build` — Full static build (must succeed, <2 min per constitution)
- `npm run start` — Local dev server (hot reload for content changes)
- `npm run swizzle` — Theme customization (if needed for code blocks)

**Checks**:
- ✅ No broken links (Docusaurus `onBrokenLinks: 'throw'` config)
- ✅ All MDX files parse correctly
- ✅ Frontmatter valid (YAML schema)
- ✅ Images/diagrams load (404 check)

### Code Example Validation (Per Chapter)
**Tools**: ROS 2 Humble colcon, pytest, Python scripts
- **Execution test**: Run each code example in `ros2_code_examples/`, capture output
- **Unit tests**: pytest for node behavior (e.g., test_publisher.py verifies message published)
- **Integration tests**: Multi-node scenarios (publisher + subscriber together)
- **URDF validation**: `check_urdf simple_humanoid.urdf` (must exit 0)

**Checks**:
- ✅ All code executes without errors on fresh ROS 2 Humble install (Ubuntu 22.04, Python 3.10)
- ✅ Output matches expected results documented in chapter
- ✅ PEP 8 compliance (`black --check`, `flake8`)
- ✅ Imports complete (no missing `import` statements)
- ✅ Docstrings present for classes and key methods

### Formatting Consistency (Automated)
**Tools**: Custom scripts, linters
- **Code block syntax**: Verify all code blocks have language tags (` ```python `, ` ```bash `, ` ```xml `)
- **Line numbers**: Consistent use of line number annotations (if enabled)
- **Indentation**: MDX content uses 2 spaces (Prettier), Python code uses 4 spaces (PEP 8)
- **Headings**: Hierarchy check (no H1 → H3 jumps)
- **Link format**: Markdown links, not raw URLs

**Checks**:
- ✅ Prettier validation (`npx prettier --check docs/module-1-ros2/**/*.mdx`)
- ✅ Consistent heading levels (script: validate-headings.js)
- ✅ Code block language tags present (script: validate-code-blocks.js)

### Technical Accuracy Validation (Manual + Automated)
**Process**:
1. **Automated**: Citation format checker (regex for `[AuthorYear]` pattern, IEEE format in bibliography)
2. **Automated**: API signature verification (compare code against rclpy documentation)
3. **Manual**: Technical peer review by ROS 2 expert (constitution requirement)
4. **Manual**: Student pilot test (2-3 students follow chapter, report issues)

**Checks**:
- ✅ Minimum 5 IEEE citations per chapter (script counts `[XYZ20XX]` references)
- ✅ Citations link to official docs (docs.ros.org, github.com/ros2)
- ✅ Code examples match official ROS 2 Humble API (no deprecated methods)
- ✅ Terminal commands accurate (`ros2 topic echo`, `ros2 service call`)
- ✅ URDF examples valid XML (check_urdf passes)
- ✅ No outdated information (version pinning: ROS 2 Humble, not "latest")

### Quality Gates (Pre-Merge)
**Gate 1: Code Execution** (blocking)
- All code examples run successfully in CI environment (GitHub Actions with ROS 2 Humble)
- Exit code 0 for all test scripts

**Gate 2: Build Success** (blocking)
- `npm run build` succeeds
- No broken links reported
- Build artifacts generated in `build/` directory

**Gate 3: Citation Count** (blocking)
- Each chapter has ≥5 IEEE-format citations
- All citations have corresponding bibliography entries

**Gate 4: Constitution Compliance** (blocking)
- Spec acceptance criteria met (SC-001 through SC-010)
- No principle violations without documented justification

**Gate 5: Peer Review** (recommended, non-blocking)
- Technical accuracy reviewed by ROS 2 expert
- Student readability test (pilot reader feedback)

## Architectural Decisions

### AD-001: Docusaurus Book Structure Style
**Decision**: Use flat chapter structure with explicit sidebar ordering, not deep nesting

**Options Considered**:
1. **Flat structure** (chosen): `docs/module-1-ros2/01-nodes.mdx`, `02-services.mdx`, `03-urdf.mdx`
   - Pros: Simple navigation, clear chapter order, easy to add/remove chapters
   - Cons: Requires manual numbering in filenames
2. **Nested structure**: `docs/module-1-ros2/01-nodes/index.mdx`, `01-nodes/exercises.mdx`
   - Pros: Organizes related content (main chapter + exercises)
   - Cons: Complicates navigation, harder to maintain single-file chapters
3. **Auto-generated sidebar**: Pure filesystem, no numbering
   - Pros: No manual ordering
   - Cons: Unpredictable sidebar order, harder to control chapter sequence

**Rationale**: Educational content benefits from explicit linear progression (Chapter 1 → 2 → 3). Flat structure with numbered filenames (`01-`, `02-`, `03-`) ensures predictable sidebar order. Single-file chapters easier to maintain (all content in one place). Aligns with Docusaurus best practices for tutorial-style documentation.

**Trade-offs Accepted**: Manual filename numbering required; renumbering needed if chapter order changes (rare for educational content).

### AD-002: Code and Diagram Formatting Approach
**Decision**: Embed code directly in MDX as fenced code blocks; diagrams as SVG files in `assets/`

**Options Considered**:
1. **Embedded code blocks** (chosen): Code inside ` ```python ` blocks in MDX
   - Pros: Syntax highlighting, copy-paste friendly, renders inline
   - Cons: Requires validation in separate workspace (can't execute from MDX)
2. **External code files**: Link to `.py` files in repo
   - Pros: Code is executable as-is
   - Cons: Readers must navigate to separate files, harder to follow narrative
3. **Interactive code playgrounds**: Embedded REPLs (e.g., Jupyter notebooks)
   - Pros: Readers can execute in browser
   - Cons: ROS 2 not browser-compatible; adds complexity; requires backend infrastructure

**For Diagrams**:
1. **SVG files** (chosen): Vector graphics in `assets/`, referenced in MDX
   - Pros: Scalable, accessible (can add alt text), lightweight
   - Cons: Requires external editor (Inkscape, Figma)
2. **Mermaid diagrams**: Inline diagram-as-code in MDX
   - Pros: Version-controlled, easy to edit
   - Cons: Limited styling, harder to create complex robotics diagrams (joint trees, coordinate frames)
3. **PNG screenshots**: Raster images from tools
   - Pros: Exact representation from simulation/tools
   - Cons: Not scalable, large file sizes, accessibility issues

**Rationale**:
- **Code**: Embedded blocks provide best reading experience (no context switching). Validation in separate `ros2_code_examples/` workspace ensures code is tested before embedding. Docusaurus Prism integration handles syntax highlighting automatically.
- **Diagrams**: SVG preferred for technical diagrams (coordinate frames, link trees) due to scalability and precision. Mermaid suitable for simple flowcharts but insufficient for robotics-specific visuals (URDF trees, transform frames). Constitution requires accessibility; SVG allows proper alt text.

**Trade-offs Accepted**: Code not directly executable from documentation (acceptable for tutorial content). Diagram creation requires external tool (one-time setup cost).

### AD-003: Sidebar and Navigation Layout
**Decision**: Auto-generated sidebar from filesystem with `_category_.json` metadata; manual top-level navigation

**Configuration**:
```typescript
// sidebars.ts (existing autogenerated approach retained)
const sidebars: SidebarsConfig = {
  tutorialSidebar: [{type: 'autogenerated', dirName: '.'}],
};
```

**`docs/module-1-ros2/_category_.json`**:
```json
{
  "label": "Module 1: ROS 2 Robotic Nervous System",
  "position": 2,
  "collapsible": true,
  "collapsed": false,
  "link": {
    "type": "generated-index",
    "description": "Learn ROS 2 fundamentals: nodes, topics, services, and URDF basics for humanoid robots."
  }
}
```

**Rationale**: Docusaurus autogeneration scales as modules added (Module 2, 3, 4). `_category_.json` provides control over sidebar label, position, and description without manual sidebar config. `collapsible: true` allows readers to expand/collapse modules. `position: 2` places Module 1 after intro page.

**Trade-offs Accepted**: Less granular control over sidebar structure (acceptable for linear tutorial content).

### AD-004: Versioning and Update Strategy
**Decision**: Single-version documentation targeting ROS 2 Humble; explicit version warnings if content updated for later ROS 2 releases

**Options Considered**:
1. **Single version** (chosen): All content for ROS 2 Humble
   - Pros: Simple, aligns with constitution version pinning, no multi-version complexity
   - Cons: Requires full rewrite for major ROS 2 updates (e.g., Iron, Jazzy)
2. **Docusaurus versioning**: Separate docs for Humble, Iron, Jazzy
   - Pros: Supports multiple ROS 2 releases simultaneously
   - Cons: Maintenance burden (update code for each version), premature optimization (no Iron/Jazzy requirement)
3. **Version-agnostic content**: Write generic ROS 2 content
   - Pros: Less maintenance
   - Cons: Violates constitution version pinning; leads to "works on my machine" issues

**Rationale**: Constitution mandates version pinning ("ROS 2 Humble, not 'latest'"). Beginner students benefit from single, tested environment (Ubuntu 22.04 + Humble). Docusaurus versioning deferred until Module 1-4 complete and later ROS 2 release required. If updates needed, use Docusaurus admonitions:

```mdx
:::caution ROS 2 Iron/Jazzy Users
This tutorial targets ROS 2 Humble. For Iron/Jazzy, see [updated tutorial](#).
:::
```

**Trade-offs Accepted**: Content lifespan limited to ROS 2 Humble LTS support window (~May 2027). Acceptable given educational focus (students use LTS releases in courses).

### AD-005: Content Authoring and Review Workflow
**Decision**: Iterative chapter-by-chapter workflow with staged gates: Draft → Code Validation → Technical Review → Merge

**Workflow Stages**:
1. **Draft** (Author): Write MDX following chapter contract, embed code placeholders
2. **Code Validation** (Author): Develop and test code in `ros2_code_examples/`, copy to MDX
3. **Build Check** (CI): Automated `npm run build`, link checker
4. **Technical Review** (Peer): ROS 2 expert validates API accuracy, citations
5. **Merge** (Maintainer): Approve PR after all gates pass

**Branching Strategy**:
- Feature branches per chapter: `001-ros2-robotic-nervous-system/chapter-1`, `/chapter-2`, `/chapter-3`
- Merge to `001-ros2-robotic-nervous-system` branch incrementally
- Final PR to `main` after Module 1 complete

**Rationale**: Incremental delivery reduces risk (each chapter independently testable). Code validation in separate workspace prevents untested code in docs. Technical review gate enforces constitution technical accuracy principle. Aligns with SDD workflow: spec → plan → tasks → implement → review.

**Trade-offs Accepted**: Slower delivery (iterative vs. batch). Acceptable for educational content requiring high accuracy.

## Risk Analysis and Mitigation

### Risk 1: ROS 2 Code Examples Fail in CI
**Probability**: Medium | **Impact**: High (blocks merge)

**Scenario**: Code examples work locally but fail in GitHub Actions CI due to environment differences (missing dependencies, timing issues).

**Mitigation**:
- Use official ROS 2 Docker images for CI (`osrf/ros:humble-desktop`)
- Document exact dependencies in `ros2_code_examples/package.xml`
- Test in isolated environment before embedding code
- Add retry logic for timing-sensitive tests (service timeouts)
- Fallback: Manual validation attestation if CI environment unfixable (document in PR)

### Risk 2: Docusaurus Build Time Exceeds Constitution Limit
**Probability**: Low | **Impact**: Medium (violates constitution)

**Scenario**: As modules added, build time exceeds 2 minutes (constitution constraint).

**Mitigation**:
- Measure build time per chapter (baseline: empty site builds in ~20s)
- Optimize images (compress SVGs, use WebP for rasters)
- Enable Docusaurus caching (`--cache` flag)
- If exceeded: Parallelize build, use incremental builds, or request constitution amendment

### Risk 3: Students Cannot Complete Chapters in Target Time
**Probability**: Medium | **Impact**: Medium (fails SC-001, SC-002, SC-010)

**Scenario**: Success criteria specify task completion times (20 min, 15 min, 10 min, 4-6 hour total). Students exceed these times.

**Mitigation**:
- Pilot test with 2-3 beginner students before finalization
- Collect feedback on pain points (unclear instructions, missing prerequisites)
- Adjust content or success criteria based on data
- Provide "Fast Track" sections for experienced students (skip basics)

### Risk 4: Citation Count Falls Short
**Probability**: Low | **Impact**: Low (easy to fix)

**Scenario**: Chapter has <5 IEEE citations (fails quality gate).

**Mitigation**:
- Automated citation counter in PR checklist
- Pre-populate bibliography template with standard ROS 2 refs (docs.ros.org, rclpy GitHub)
- Review checklist item: "Count citations (target: ≥5)"

### Risk 5: Diagrams Become Outdated
**Probability**: Medium | **Impact**: Low (maintenance burden)

**Scenario**: ROS 2 tools change (e.g., `urdf_to_graphiz` output format), diagrams no longer match reality.

**Mitigation**:
- Store diagram source files (`.svg` sources, not exports)
- Document diagram creation process in `quickstart.md`
- Periodic review (quarterly) to validate diagrams against latest tools
- Version diagram filenames if major changes needed (`urdf-tree-v2.svg`)

## Success Metrics (Mapped to Spec)

**From spec.md Success Criteria**:
- **SC-001**: Student creates publisher-subscriber pair in <20 min → Measure: Pilot test timing
- **SC-002**: Student calls service in <15 min → Measure: Pilot test timing
- **SC-003**: Student reads 10-link URDF in <10 min → Measure: Pilot test timing
- **SC-004**: Student modifies URDF in <5 min → Measure: Exercise completion time
- **SC-005**: 90% exercise success rate → Measure: Self-assessment survey
- **SC-006**: All code runs on fresh ROS 2 Humble → Measure: CI test pass rate
- **SC-007**: ≥5 citations per chapter → Measure: Automated count
- **SC-008**: 80% comprehension quiz pass → Measure: Quiz results (future)
- **SC-009**: Student traces message flow in <5 min → Measure: Exercise timing
- **SC-010**: 4-6 hour completion time → Measure: Pilot test total time

**Additional Plan Metrics**:
- Docusaurus build time: <2 minutes (constitution)
- Page load time: <3s on 3G (constitution)
- Code validation test pass rate: 100% (all examples execute)
- Link checker pass rate: 100% (no 404s)
- PEP 8 compliance: 100% (all Python code)

## Next Steps

1. **Review and Approve Plan**: Obtain user approval for architectural decisions
2. **Execute Phase 0**: Create `research.md` (Docusaurus patterns, ROS 2 API)
3. **Execute Phase 1**: Create `data-model.md`, `contracts/`, `quickstart.md`
4. **Run `/sp.tasks`**: Generate dependency-ordered task list
5. **Implement Chapters**: Follow iterative workflow (Draft → Validate → Review → Merge)
6. **Final Validation**: Full constitution compliance check, success criteria verification
7. **Deploy**: Merge to main, trigger GitHub Pages deployment
