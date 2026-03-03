# EEM 413/414 Revised Project Plan
## Real-time Path Following Control of a Skid-Steer Mobile Robot  
**Autonomous Navigation in ROS 2 with MATLAB Parameter Interface**

**Project Duration:** February 9, 2026 - May 22, 2026 (15 weeks)

**Team Members:**
- **Necati Durmuş** - ROS Navigation & Hardware Lead
- **Zeynep Selin Hatunoğlu** - Integration & MATLAB Interface Lead

**Advisor:** Asst. Prof. Hüseyin Ersin EROL

---

## Simplified Project Scope

### Core Architecture (Revised)
- **ROS 2 Nav2:** Handles ALL navigation, path planning, obstacle avoidance, and control
- **MATLAB/Simulink:** Mission planning tool for:
  - Defining waypoint trajectories (GPS coordinates or map positions)
  - Setting navigation parameters (max speed, braking distance, safety margins)
  - Monitoring and visualizing robot performance
- **Gazebo:** Simulation environment for testing
- **Real Hardware:** Final deployment platform (if available)

### Project Goals
1. **Autonomous navigation** with waypoint following and obstacle avoidance
2. **Mission planning interface** in MATLAB for trajectory definition and parameter tuning
3. **Simulation-based validation** (primary deliverable)
4. **Hardware demonstration** (optional, if budget and resources permit)

---

## Work Distribution

### Necati Durmuş - Navigation & Hardware (60%)
**Primary Responsibilities:**
- ROS 2 environment setup (Gazebo, Leo Rover, sensors)
- Nav2 stack configuration and tuning
- Autonomous navigation development
- Hardware deployment and testing

### Zeynep Selin Hatunoğlu - Integration & Interface (40%)
**Primary Responsibilities:**
- MATLAB-ROS 2 interface development
- Mission planning tool in MATLAB
- Parameter configuration system
- System integration and testing support

---

## Timeline Overview

| Phase | Weeks | Focus | Lead |
|-------|-------|-------|------|
| **1. Foundation** | 1-2 (Feb 9-22) | ROS setup + MATLAB interface | Both |
| **2. Navigation Development** | 3-6 (Feb 23-Mar 22) | Nav2 configuration | Necati |
| **3. Mission Planning** | 5-8 (Mar 9-Apr 5) | MATLAB tools | Zeynep |
| **4. Integration & Testing** | 9-12 (Apr 6-May 3) | Complete system | Both |
| **5. Thesis Preparation** | 13-15 (May 4-22) | Optional hardware + thesis writing | Both |
| **6. Thesis Defense** | June 8-12 | Final defense | Both |

---

## Detailed Work Packages

### WP1: System Setup (Weeks 1-2: Feb 9-22)

#### Necati's Tasks:
- Install Ubuntu 22.04 + ROS 2 Humble
- Deploy Leo Rover model in Gazebo
- Configure sensors (LiDAR, IMU, odometry)
- Verify sensor data streams
- Test basic teleoperation

**Deliverable:** Functional Gazebo simulation with sensor topics

#### Zeynep's Tasks:
- Install MATLAB with ROS 2 Toolbox
- Test basic ROS-MATLAB communication
- Subscribe to sensor topics from MATLAB
- Publish test commands to robot
- Document topic interfaces

**Deliverable:** Working MATLAB-ROS connection

**Joint Success Criteria:**
- Robot moves in Gazebo via keyboard
- MATLAB can read sensor data
- MATLAB can send velocity commands

---

### WP2: Navigation Stack Development (Weeks 3-6: Feb 23-Mar 22)

#### Necati's Tasks (Independent):
- Install and configure Nav2 packages
- Set up map server and AMCL localization
- Configure global planner (Smac or NavFn)
- Configure local planner (DWB or Regulated Pure Pursuit)
- Tune navigation parameters:
  - Maximum velocity limits
  - Acceleration limits
  - Obstacle inflation radius
  - Local costmap size
- Create test scenarios and maps
- Test autonomous navigation (ROS-only)

**Deliverables:**
- Configured Nav2 stack
- Parameter files (YAML)
- 5 test mission files
- Navigation performance report

**Success Criteria:**
- Robot reaches goal in empty map (>90% success)
- Robot avoids obstacles (>70% success)
- Path planning completes in <1 second

**Weekly Targets:**
- Week 3: Nav2 installed, basic configuration
- Week 4: Planners configured, initial testing
- Week 5: Parameter tuning, obstacle avoidance working
- Week 6: Final tuning, mission files created

---

### WP3: Mission Planning Interface (Weeks 5-8: Mar 9-Apr 5)

#### Zeynep's Tasks (Parallel to WP2):
- Design MATLAB mission planning GUI
- Implement waypoint definition system:
  - Manual waypoint entry (x, y coordinates)
  - Interactive map-based selection
  - Import from GPS coordinates
- Create parameter configuration interface:
  - Maximum speed setting
  - Braking reaction time (BRT)
  - Minimum safe distance to obstacles
  - Distance-to-goal threshold
- Implement mission file generation (YAML format for Nav2)
- Create ROS 2 mission launcher from MATLAB
- Build performance monitoring dashboard

**Deliverables:**
- MATLAB mission planning GUI
- Parameter configuration tool
- Mission-to-Nav2 converter
- Real-time monitoring dashboard

**Success Criteria:**
- GUI allows easy waypoint definition
- Parameters correctly modify Nav2 behavior
- Generated missions work in Nav2
- Dashboard displays robot status in real-time

**Weekly Targets:**
- Week 5: GUI design, basic waypoint input
- Week 6: Parameter interface, YAML generation
- Week 7: Mission launcher integration
- Week 8: Monitoring dashboard, testing

---

### WP4: System Integration (Weeks 9-10: Apr 6-19)

#### Joint Tasks:
- Integrate MATLAB mission planner with Nav2
- Test complete workflow:
  1. Define mission in MATLAB
  2. Launch navigation in ROS
  3. Monitor execution in MATLAB
  4. Log results
- Verify parameter changes affect robot behavior
- Debug integration issues
- Optimize performance

**Division of Labor:**
- **Necati:** Ensure Nav2 accepts MATLAB-generated missions
- **Zeynep:** Ensure MATLAB correctly interfaces with ROS

**Deliverables:**
- End-to-end working system
- Integration test results
- User workflow documentation

**Success Criteria:**
- Mission defined in MATLAB executes successfully
- Parameters set in MATLAB control robot behavior
- System runs 30+ minutes without crashes

---

### WP5: Comprehensive Testing (Weeks 11-12: Apr 20-May 3)

#### Test Campaign (Both Students):

**Test Scenarios:**
1. Empty environment (straight paths) - 5 runs
2. Corridor navigation (turns) - 5 runs  
3. Cluttered environment (obstacles) - 10 runs
4. Multi-waypoint missions (5+ waypoints) - 5 runs
5. Parameter variation tests - 10 runs

**Testing Protocol:**
- Create missions in MATLAB
- Execute in simulation
- Log all data (paths, velocities, errors)
- Analyze performance
- Document failures

**Deliverables:**
- Test results table (success rates)
- Performance metrics (path error, completion time)
- Parameter sensitivity analysis
- Failure mode classification
- 3-5 demonstration videos

**Success Criteria:**
- 90% success in empty environment
- 70% success with obstacles
- Clear documentation of failure modes
- Parameter effects quantified

---

### WP6: Hardware Deployment (OPTIONAL - Weeks 13-14: May 4-17)

**NOTE:** Hardware testing is optional and depends on budget availability and robot readiness. Primary deliverable is simulation-based validation.

**If Hardware Available and Ready:**

#### Week 13: Preparation
- **Necati:** Secure robot platform, install onboard computer, configure sensors
- **Zeynep:** Install ROS 2 on robot, transfer software

#### Week 14: Testing
- **Necati:** Map test environment, conduct hardware tests
- **Zeynep:** Deploy MATLAB interface, tune parameters for hardware
- **Both:** Record demonstration materials

**Deliverables (if hardware attempted):**
- Hardware setup documentation
- Real-world test results
- Comparison with simulation results

---

**Default Plan (No Hardware or Hardware Not Ready):**

Use Weeks 13-14 for:
- **Enhanced simulation testing** (additional scenarios, statistical validation)
- **Thesis writing** (early start on documentation)
- **Competition preparation** (if participating in faculty competition)
- **Polish demonstration materials** for thesis defense

---

### WP7: Thesis Preparation & Optional Competition (Week 15: May 18-22)

#### Graduation Thesis Requirements

**Thesis Document (60-80 pages):**
1. **Introduction** (10 pages)
   - Problem definition
   - Literature review
   - Project objectives and scope
   
2. **System Design** (15-20 pages)
   - System architecture
   - ROS 2 navigation stack design
   - MATLAB interface design
   - Integration methodology

3. **Implementation** (15-20 pages)
   - Navigation configuration details
   - Mission planning tool development
   - Testing framework

4. **Results and Analysis** (15-20 pages)
   - Simulation test results
   - Performance metrics and analysis
   - Hardware results (if applicable)
   - Discussion of findings

5. **Conclusion and Future Work** (5-10 pages)
   - Summary of achievements
   - Lessons learned
   - Recommendations for future work

6. **References and Appendices**
   - Bibliography
   - Configuration files
   - User manuals

**Division of Writing:**
- **Necati:** Chapters on system architecture, navigation implementation, hardware (if applicable)
- **Zeynep:** Chapters on MATLAB interface, integration, testing methodology
- **Both:** Introduction, results, conclusion (collaborative)

**Thesis Defense (June 8-12, 2026):**
- 20-minute presentation
- 15-minute Q&A with committee
- Demonstration of working system (live or video)

---

#### Optional: Faculty Graduation Project Competition

**Note:** Competition requirements will be announced in early May. If you wish to participate, plan for 2-week preparation window.

**Typical Requirements (tentative):**
- Competition poster (A0 or A1 size)
- 5-minute presentation
- Live demonstration or video
- Project summary (2-4 pages)

**Timeline if Participating:**
- **Early May:** Requirements announced by faculty
- **Within 2 weeks:** Prepare and submit materials to Dean's Office
- **Mid-May:** Competition presentation

**Recommendation:** Decide by May 3 whether to participate based on project progress.

---

## Graduation Thesis Requirements (Detailed)

### Thesis Document Structure (60-80 pages)

**Front Matter:**
- Title page
- Abstract (Turkish and English, 1 page each)
- Acknowledgments
- Table of contents
- List of figures
- List of tables
- List of abbreviations

**Main Content:**

**Chapter 1: Introduction (8-10 pages)**
- Background and motivation
- Problem statement
- Project objectives
- Scope and limitations
- Thesis organization

**Chapter 2: Literature Review (10-12 pages)**
- Mobile robot navigation (ROS 2, Nav2)
- Skid-steer kinematics and control challenges
- Mission planning approaches
- MATLAB-ROS integration methods
- Summary and gap analysis

**Chapter 3: System Design and Architecture (12-15 pages)**
- Overall system architecture diagram
- Hardware components (sensors, actuators, computers)
- Software architecture (ROS 2 nodes, MATLAB modules)
- Communication interfaces
- Design decisions and rationale

**Chapter 4: Implementation (15-20 pages)**

*Necati's primary responsibility:*
- ROS 2 environment setup
- Gazebo simulation configuration
- Nav2 stack implementation
- Navigation algorithms and parameters
- Test scenario development

*Zeynep's primary responsibility:*
- MATLAB-ROS 2 interface development
- Mission planning GUI implementation
- Parameter configuration system
- Monitoring dashboard
- Integration methodology

**Chapter 5: Testing and Results (12-15 pages)**

*Both students collaborate:*
- Testing methodology
- Simulation test scenarios
- Performance metrics (success rates, path errors, timing)
- Statistical analysis of results
- Hardware testing results (if applicable)
- Comparison and discussion

**Chapter 6: Conclusion and Future Work (5-7 pages)**
- Summary of achievements
- Contributions
- Lessons learned
- Limitations
- Future work recommendations

**Back Matter:**
- References (IEEE style, 30+ citations)
- Appendices:
  - Configuration files
  - User manuals
  - Code documentation
  - Additional test results

---

### Writing Timeline

| Period | Writing Tasks |
|--------|---------------|
| **Feb-Mar** | Literature review (Chapter 2), keep notes on implementation |
| **Apr** | System design (Chapter 3), document implementation as you build (Chapter 4) |
| **May 1-10** | Complete results (Chapter 5), draft introduction and conclusion |
| **May 11-20** | Finalize all chapters, format document, polish writing |
| **May 21-31** | Final review with advisor, incorporate feedback, proofread |
| **Jun 1-7** | Submit final version, prepare defense presentation |

---

### Thesis Defense Presentation (June 8-12)

**Presentation Structure (20 minutes):**
1. **Introduction (3 min)**
   - Necati: Problem motivation, objectives, project scope
   
2. **System Architecture (3 min)**
   - Zeynep: Overall architecture diagram, key components

3. **Navigation System (5 min)**
   - Necati: ROS 2 Nav2 implementation, configuration, algorithms

4. **Mission Planning Interface (4 min)**
   - Zeynep: MATLAB GUI, parameter system, integration

5. **Results and Demonstration (4 min)**
   - Both: Key results, live demo or video, performance analysis

6. **Conclusion (1 min)**
   - Both: Summary, contributions, future work

**Q&A Session (15 minutes):**
- Be prepared to answer technical questions
- Explain design decisions
- Discuss limitations and alternatives
- Demonstrate deep understanding

---

---

## Weekly Schedule

**Biweekly Advisor Meetings:** Every other Monday at 12:00 PM (30 minutes)

| Week | Dates | Necati | Zeynep | Meeting | Special Deadlines |
|------|-------|--------|--------|---------|-------------------|
| 1 | Feb 9-15 | WP1: Gazebo setup | WP1: MATLAB-ROS setup | Mon 12:00 | - |
| 2 | Feb 16-22 | WP1: Sensors + test | WP1: Interface testing | - | - |
| 3 | Feb 23-Mar 1 | WP2: Nav2 install | WP3: GUI design start | Mon 12:00 | - |
| 4 | Mar 2-8 | WP2: Planner config | WP3: GUI development | - | - |
| 5 | Mar 9-15 | WP2: Parameter tuning | WP3: Parameter interface | Mon 12:00 | - |
| 6 | Mar 16-22 | WP2: Testing + missions | WP3: YAML generation | - | - |
| 7 | Mar 23-29 | Support Zeynep | WP3: Mission launcher | Mon 12:00 | **Mar 28: Progress Report 1 Due** |
| 8 | Mar 30-Apr 5 | Nav2 final validation | WP3: Dashboard + test | - | - |
| 9 | Apr 6-12 | WP4: Integration | WP4: Integration | Mon 12:00 | - |
| 10 | Apr 13-19 | WP4: System testing | WP4: System testing | - | - |
| 11 | Apr 20-26 | WP5: Run test campaign | WP5: Log and analyze | Mon 12:00 | - |
| 12 | Apr 27-May 3 | WP5: Complete tests | WP5: Document results | - | **May 2: Progress Report 2 Due** |
| 13 | May 4-10 | WP6: Hardware (optional) or thesis writing | WP6: Hardware (optional) or thesis writing | Mon 12:00 | **Early May: Competition announced** |
| 14 | May 11-17 | WP6: Continue work + thesis | WP6: Continue work + thesis | - | **Competition submission (~2 weeks after announcement)** |
| 15 | May 18-22 | WP7: Thesis finalization | WP7: Thesis finalization | Mon 12:00 | - |
| 16 | May 25-31 | Thesis submission preparation | Thesis submission preparation | - | - |
| 17 | Jun 1-7 | Defense preparation | Defense preparation | Mon 12:00 | - |
| 18 | Jun 8-12 | **THESIS DEFENSE** | **THESIS DEFENSE** | - | **Jun 8-12: Defense Week** |

**Meeting Schedule Notes:**
- Weeks with meetings: 1, 3, 5, 7, 9, 11, 13, 15, 17
- Weeks without meetings: 2, 4, 6, 8, 10, 12, 14, 16
- Students should prepare brief progress updates for each meeting
- Additional meetings can be scheduled if urgent issues arise

---

## Key Milestones & Deadlines

| Date | Milestone | Owner | Type |
|------|-----------|-------|------|
| Feb 22 | System setup complete | Both | Internal |
| Mar 22 | Nav2 working autonomously | Necati | Internal |
| **Mar 28** | **Progress Report 1 Due** | **Both** | **MANDATORY** |
| Apr 5 | MATLAB mission planner complete | Zeynep | Internal |
| Apr 19 | Integrated system working | Both | Internal |
| **May 2** | **Progress Report 2 Due** | **Both** | **MANDATORY** |
| May 3 | Testing complete, hardware decision | Both | Internal |
| Early May | Faculty competition announced | Both | Optional |
| ~May 15 | Competition submission deadline | Both | Optional |
| May 22 | Thesis draft complete | Both | Internal |
| **Jun 8-12** | **Thesis Defense** | **Both** | **MANDATORY** |

---

## Mandatory Progress Reports

Students must complete and submit progress reports via the department's online system at two checkpoints:

### Progress Report 1 - Due: March 28, 2026

**Required Content:**
- Work completed in Weeks 1-7
- Current system status
- Challenges encountered and solutions
- Deviation from plan (if any)
- Plan for remaining work

**What to Report:**
- **Necati:** ROS environment setup, Nav2 configuration progress, navigation test results
- **Zeynep:** MATLAB-ROS interface status, mission planning GUI development progress
- **Both:** Integration status, any blockers or delays

**Access:** Department online page (link TBD by department)

---

### Progress Report 2 - Due: May 2, 2026

**Required Content:**
- Work completed in Weeks 8-12
- Complete system testing results
- Performance metrics and analysis
- Hardware status (attempted/not attempted, reasons)
- Competition participation decision
- Remaining work for thesis

**What to Report:**
- **Necati:** Final Nav2 configuration, complete test results, hardware preparation status
- **Zeynep:** Complete MATLAB interface, integration results, monitoring dashboard
- **Both:** System performance metrics, failure analysis, thesis preparation status

**Access:** Department online page (link TBD by department)

---

## Optional Faculty Graduation Project Competition

**Timeline:**
- **Early May 2026:** Competition requirements announced by Dean's Office
- **~2 weeks after announcement:** Submission deadline
- **Mid-May:** Competition presentations

**Typical Requirements (subject to change):**
- Project poster (A0 or A1 format)
- Short presentation (5-10 minutes)
- Live demonstration or video
- Project summary document (2-4 pages)

**Decision Point:** By May 3, discuss with advisor whether to participate based on:
- Project completion status
- Quality of results
- Available time for preparation
- Student interest and motivation

**If Participating:**
- Allocate time in Weeks 13-14 for competition materials
- Coordinate with advisor for poster/presentation review
- Prepare compelling demonstration

**Note:** Participation is voluntary and should not compromise thesis quality.

---

## Success Metrics & Thesis Evaluation Criteria

### Minimum Success (Pass Thesis Defense)
**Technical Achievement:**
- ✓ Autonomous navigation works in simulation
- ✓ MATLAB mission planner functional with basic features
- ✓ Both progress reports submitted on time
- ✓ Complete thesis document (60+ pages)
- ✓ Successful defense presentation

**Documentation:**
- ✓ System architecture documented
- ✓ Implementation details explained
- ✓ Test results reported
- ✓ All code accessible and documented

---

### Good Success (Strong Pass)
**Technical Achievement:**
- ✓ 70%+ success rate in simulation tests
- ✓ Functional MATLAB GUI with all planned features
- ✓ Comprehensive testing across multiple scenarios
- ✓ Hardware attempted (even if limited success)
- ✓ Well-written thesis with clear analysis

**Documentation:**
- ✓ Professional thesis quality
- ✓ Clear presentation delivery
- ✓ Good response to committee questions
- ✓ Publication-quality figures and results

---

### Excellent Success (Honors/High Distinction)
**Technical Achievement:**
- ✓ 90%+ success rate in simulation
- ✓ Polished MATLAB interface with advanced features
- ✓ Successful hardware deployment with analysis
- ✓ Participation in faculty competition
- ✓ Outstanding thesis with comprehensive analysis

**Documentation:**
- ✓ Exceptional thesis quality (publication-ready)
- ✓ Outstanding presentation and defense
- ✓ Impressive demonstration
- ✓ Significant technical contributions
- ✓ Strong grasp of all technical concepts

---

### Evaluation Committee Will Assess:
1. **Technical Merit (40%)**: System functionality, innovation, complexity
2. **Implementation Quality (20%)**: Code quality, documentation, testing
3. **Thesis Document (20%)**: Writing quality, organization, technical depth
4. **Presentation & Defense (15%)**: Communication, understanding, Q&A responses
5. **Project Management (5%)**: Timeline adherence, progress reports, professionalism

---

## Risk Management

| Risk | Mitigation | Backup Plan |
|------|------------|-------------|
| Nav2 configuration difficult | Start early, use online resources, ask advisor | Use default parameters, simpler scenarios |
| MATLAB-ROS integration issues | Test early in WP1, use official examples | Simplify interface, manual mission files |
| Hardware unavailable | Plan alternative at Week 12 | Extended simulation, statistical analysis |
| Time constraints | Weekly monitoring, clear priorities | Reduce scope: focus on simulation only |
| One student unavailable | Cross-training, documentation | Other student continues with reduced scope |

---

## Communication Protocol

**Biweekly Advisor Meetings:** Every other Monday at 12:00 PM (30 minutes)
- Weeks 1, 3, 5, 7, 9, 11, 13, 15, 17
- Come prepared with progress updates
- Bring specific questions or blockers
- Discuss next two weeks' plan

**Meeting Preparation:**
- Each student prepares 5-minute update
- Show demonstrations of completed work
- List any blocking issues
- Propose solutions if possible

**Asynchronous Communication:**
- **WhatsApp/Email:** Quick questions and updates
- **Google Drive:** File sharing and documentation
- **Git Repository:** Code sharing and version control
- **Response Time:** Within 24-48 hours

**Emergency Contact:**
- Critical blockers: Contact advisor immediately
- Don't wait for scheduled meeting if stuck
- Email or WhatsApp for urgent issues

---

## Individual Responsibilities Summary

### Necati Durmuş - What You Need to Deliver

**Technical Implementation:**
- Functional Gazebo simulation environment
- Configured Nav2 with tuned parameters
- Test scenarios and mission files
- Navigation performance results and analysis
- Hardware deployment (if attempted)

**Documentation & Writing:**
- Nav2 configuration guide and parameter rationale
- Test results and failure analysis documentation
- Hardware setup instructions (if applicable)
- **Thesis Chapters:** System architecture (navigation focus), ROS 2 implementation details, hardware sections
- **Progress Reports:** ROS and navigation sections

**Weekly Reports (Biweekly Meetings):**
- Navigation development progress and challenges
- Test results (success rates, failure modes)
- Parameter tuning status and rationale
- Hardware readiness and timeline

---

### Zeynep Selin Hatunoğlu - What You Need to Deliver

**Technical Implementation:**
- MATLAB-ROS 2 communication interface
- Mission planning GUI (waypoint definition, parameter settings)
- Parameter configuration system
- Performance monitoring dashboard
- Complete integration with Nav2

**Documentation & Writing:**
- MATLAB interface user guide and feature descriptions
- Parameter descriptions and effects on navigation
- Integration methodology documentation
- Testing procedures and results compilation
- **Thesis Chapters:** MATLAB interface design, integration methodology, testing and results analysis
- **Progress Reports:** MATLAB and integration sections

**Weekly Reports (Biweekly Meetings):**
- Interface development progress
- MATLAB-ROS integration status and challenges
- GUI functionality updates and user testing
- Testing coordination and results

---

---

## Thesis Submission Process

### Important Dates and Procedures

**Thesis Submission:**
- **Deadline:** TBD by department (typically 1-2 weeks before defense)
- **Format:** PDF + printed copies (check department requirements)
- **Submission:** Via department online system
- **Copies Required:** Typically 3 printed + bound copies

**Defense Scheduling:**
- **Defense Period:** June 8-12, 2026
- **Exact Date/Time:** Will be announced by department
- **Location:** TBD by department
- **Committee:** 3-5 faculty members

**Required Approvals:**
- Advisor approval before submission
- Department format check
- Plagiarism check (via Turnitin or similar)

**Formatting Requirements (verify with department):**
- Font: Times New Roman, 12pt
- Spacing: 1.5 or double spacing
- Margins: 2.5cm all sides (or per department template)
- Page numbering: Required
- Citation style: IEEE or APA (confirm with advisor)

**Student Responsibilities:**
- Keep advisor informed of progress
- Submit drafts for review with sufficient lead time
- Address all feedback before final submission
- Check all department requirements and deadlines
- Attend defense on scheduled date/time

---

## Final Checklist

### By Project End, You Must Have:

**Mandatory Deliverables:**

**Both Students:**
- [ ] Progress Report 1 submitted (March 28)
- [ ] Progress Report 2 submitted (May 2)
- [ ] Autonomous navigation working in simulation
- [ ] MATLAB mission planner functional
- [ ] Complete graduation thesis (60-80 pages)
- [ ] Thesis defense presentation (20 minutes)
- [ ] System demonstration (live or video)
- [ ] All code on GitHub with documentation

**Necati:**
- [ ] Nav2 fully configured and documented
- [ ] Test scenarios and results
- [ ] Navigation performance analysis
- [ ] Hardware documentation (if attempted)
- [ ] Thesis chapters on navigation and system architecture

**Zeynep:**
- [ ] MATLAB GUI fully functional
- [ ] All interface features implemented
- [ ] Integration validated and documented
- [ ] User manual for MATLAB interface
- [ ] Thesis chapters on interface and testing methodology

---

**Optional Deliverables:**

**Hardware Deployment (if resources permit):**
- [ ] Robot platform configured
- [ ] Real-world navigation tests
- [ ] Sim-to-real comparison

**Faculty Competition (if participating):**
- [ ] Competition poster (A0/A1)
- [ ] Competition presentation (5-10 min)
- [ ] Competition summary document
- [ ] Submitted to Dean's Office on time

---

### Thesis Defense Preparation Checklist (June 8-12)

**4 Weeks Before Defense:**
- [ ] Complete thesis draft
- [ ] Share with advisor for review
- [ ] Begin presentation preparation

**2 Weeks Before Defense:**
- [ ] Incorporate advisor feedback
- [ ] Finalize thesis document
- [ ] Complete presentation slides
- [ ] Prepare demonstration

**1 Week Before Defense:**
- [ ] Submit final thesis to department
- [ ] Rehearse presentation (multiple times)
- [ ] Test demonstration thoroughly
- [ ] Prepare for potential questions

**Defense Day:**
- [ ] Arrive 30 minutes early
- [ ] Test equipment and demo
- [ ] Backup materials ready (USB, video)
- [ ] Professional appearance
- [ ] Confident delivery

---

## MATLAB Interface Requirements (Reference)

### Essential Features:
1. **Waypoint Definition**
   - Manual entry (x, y, orientation)
   - Map-based point-and-click
   - Import from file (CSV, GPS coordinates)

2. **Parameter Configuration**
   - Max velocity (m/s)
   - Max acceleration (m/s²)
   - Braking reaction time (s)
   - Minimum safe distance (m)
   - Goal tolerance (m)

3. **Mission Management**
   - Save/load missions
   - Generate Nav2 YAML files
   - Launch navigation from MATLAB

4. **Monitoring (Optional but Recommended)**
   - Real-time position plot
   - Current velocity display
   - Distance to goal
   - Obstacle proximity alerts

---

## Remember

✅ **Mark mandatory deadlines** - March 28 and May 2 progress reports are non-negotiable  
✅ **Hardware is optional** - Don't stress if robot unavailable; simulation is sufficient  
✅ **Communicate regularly** - Only two of you, coordination is critical  
✅ **Prepare for biweekly meetings** - Come with updates and questions  
✅ **Start thesis early** - Don't wait until May; write as you develop  
✅ **Test incrementally** - Catch problems early, not during integration  
✅ **Document everything** - You'll need it for progress reports and thesis  
✅ **Ask for help** - Advisor is here to support; don't struggle alone  
✅ **Consider competition** - Optional but great experience if time permits  
✅ **Focus on defense** - June 8-12 is the ultimate deadline  

**You can do this! Stay organized, communicate well, and deliver incrementally.**

---

*Document Version: 2.0 (Revised for 2-person team with thesis requirements)*  
*Last Updated: February 2026*  
*Biweekly Meetings: Every other Monday at 12:00 PM*  
*Contact: Asst. Prof. Hüseyin Ersin EROL*