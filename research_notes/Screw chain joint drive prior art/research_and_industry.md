# Research and industrial prior art: a screw nut pulling a wrapped tension member to turn a joint

Reference mechanism (Mike's arm, 2026): an SFU1204 ball screw (12 mm, 4 mm lead) lies along the link and is driven by a NEMA 17 through a 1:1 belt. The ball nut holds two #25 roller chains, one on each side of the screw. Each chain runs from the nut, around a 25-tooth sprocket fixed to the next joint, around a small idler, and back to the nut. About 64 mm of nut travel gives about 145 degrees. The motor and screw ride on the link, and the whole reduction is steel.

Method note: primary PDFs were read in full text where possible (Pratt 2000 thesis, Paine & Sentis UT-SEA paper, Edsinger-Gonzales & Weber Domo paper, Jarrasse/Garrec IROS 2008 paper, Bridgwater et al. R2 hand paper). Items marked "(search summary only)" were seen only in search-engine snippets of the named source. They are not verified against the full text.

## Q1. Which research robots use a ball screw or lead screw whose nut pulls a cable, belt, band or chain around a joint pulley or drum?

### Takeaway
Ball screw, then cable, then joint pulley was a standard electric legged-robot drive at the MIT Leg Lab and MIT CSAIL in the 1990s and 2000s (Spring Flamingo, Domo, and per one review the ARL Monopod). CEA-LIST in France patented and productised a screw-and-cable version (the SCS actuator: the ABLE exoskeleton and Virtuose haptic arms). The MIT agonist-antagonist knee prosthesis and the NASA/GM Robonaut 2 hand use the same idea. Spring Flamingo is the closest by far: a belt-driven ball screw whose output pulls a cable over an idler to a joint pulley, at roughly 110:1 to 160:1. Every example found uses steel or polymer cable (or tendon). None uses roller chain.

### Cited Findings

**Spring Flamingo, MIT Leg Lab (Jerry Pratt, PhD thesis 2000; robot from the late 1990s). Closest match found.**
- The robot has six identical Series Elastic Actuators, all mounted in the upper body. Cable drives carry the power to the hip, knee and ankle. — [Pratt 2000 thesis, App. A](https://www.ftp.ai.mit.edu/pub/users/jpratt/phd_thesis.pdf)
- Actuator chain: a brushed DC motor, a timing belt with 2.5:1 reduction, then a ball screw listed as "0.125 in/rev". The ball nut pushes a block onto four die springs, and the springs push carbon-fibre tubes that carry the output. "The output of the actuator is attached to a cable which winds around an idler pulley and travels to the joint." — [Pratt 2000 thesis, A.3](https://www.ftp.ai.mit.edu/pub/users/jpratt/phd_thesis.pdf)
- Joint pulleys: 1.75 in (44 mm) diameter at the hip and ankle, 2.5 in (64 mm) at the knee. Rated 647 N and 0.221 m/s at the actuator, giving 14.4 Nm and 9.9 rad/s at hip and ankle and 20.6 Nm and 6.9 rad/s at the knee. The pulleys turn on bearings on shafts fixed to the lower link of each joint. Cables bound for the knee and ankle pass over 1.2 in pulleys at the hip. — [Pratt 2000 thesis, A.1–A.3](https://www.ftp.ai.mit.edu/pub/users/jpratt/phd_thesis.pdf)
- Range-of-motion figure (A-7) labels, as extracted: hip about ±45°, knee about 90°, ankle about +15°/−45°. The extraction is garbled, so treat these as approximate. — [Pratt 2000 thesis, Fig. A-7](https://www.ftp.ai.mit.edu/pub/users/jpratt/phd_thesis.pdf)
- The thesis text also prints the lead as "0.3175mm/rev". Its own force and speed numbers only work with 3.175 mm (0.125 in), so 0.3175 is a typo. — [Pratt 2000 thesis, A.3](https://www.ftp.ai.mit.edu/pub/users/jpratt/phd_thesis.pdf)
- The cable material is not given in the sections I read. Spring Flamingo's 1998 ICRA paper is "Intuitive control of a planar bipedal walking robot" (J. Pratt and G. Pratt). — cited as ref [17] in [Paine & Sentis](https://sites.utexas.edu/hcrl/wp-content/uploads/sites/3888/2016/01/prismatic-sea-paine.pdf)

**Domo humanoid, MIT CSAIL (Edsinger-Gonzales & Weber; IEEE Humanoids 2004, later in the Int. J. Humanoid Robotics).**
- The arm uses a "cable-drive SEA": a brushless DC motor drives an inner carriage through a precision ball screw. Die springs pass that motion to an outer carriage, and a cable transmission takes it to the joint. — [Edsinger-Gonzales & Weber, Domo paper](https://groups.csail.mit.edu/lbr/mars/pubs/humanoids04.pdf)
- The authors' trade-offs: the ball screw gives better efficiency and shock tolerance than a gearhead, but its travel limits the actuator and makes packaging hard. Cable breakage at the crimps was a known failure. Arm springs are 300 lb/in. The arms are "based on a cable-drive design similar to that of the WAM arm". — [Domo paper](https://groups.csail.mit.edu/lbr/mars/pubs/humanoids04.pdf)

**CEA-LIST Screw-and-Cable System (SCS) (Philippe Garrec; French patent FR0101630, 2000, also filed as US 10/296,740). Used in the ABLE upper-limb exoskeleton (IROS 2008) and CEA/Haption haptic arms.**
- SCS principle: the hollow screw is locked against rotation, and the nut turns in a bearing. The screw translates with no linear guide; a pair of rollers in a slot, joined to the screw by a flexible coupling, keeps it from turning. The transmission cable passes through the screw's bore and is attached at its middle. — [Jarrasse, Robertson, Garrec et al., IROS 2008](https://infoscience.epfl.ch/record/181352/files/04651014-1.pdf?version=1)
- Claimed advantages: high force capacity from a high overall ratio, a low friction threshold, high backdrivability, the motor parallel to the cable (compact), low inertia, high stiffness, and tolerance of structural flex. The SCS is embedded in the limb segment "like electrical muscles". Keeping the cable loop inside the limb shortens the cable and stiffens the drive. — [IROS 2008 paper](https://infoscience.epfl.ch/record/181352/files/04651014-1.pdf?version=1)
- ABLE specifications: 4 actuated axes (3 at the shoulder, 1 at the elbow); arm mass with motors 2.3 kg; Faulhaber DC motors; transmission "Ball-screw and cable (SCS)"; Cartesian speed over 1 m/s; continuous joint torque 18 Nm on axes 1–2 and 13 Nm on axis 3 (the elbow column was partly unreadable in my extraction); a 110° amplitude is listed for one axis. — [IROS 2008 paper, Table 2](https://infoscience.epfl.ch/record/181352/files/04651014-1.pdf?version=1)
- ABLE's own paper (Garrec et al., IROS 2008) reportedly describes it as a 4-axis exoskeleton driven by SCS actuators, and says a master arm was industrialised by Haption as the Virtuose 6D 4040 (search summary only; the HAL PDF blocked automated access). — [HAL record cea-01588401](https://hal.archives-ouvertes.fr/cea-01588401)
- Later Garrec patents continue the family: a screw turning in a frame, a nut, and two cables coupled to the nut, with force estimated from the nut's angular movement (US 12,270,459, 2025). — [USPTO Gazette US12270459](https://patentsgazette.uspto.gov/week14/OG/html/1533-2/US12270459-20250408.html) (search summary only)

**MIT agonist-antagonist active knee prosthesis (E. Martinez-Villalpando and H. Herr, MIT Media Lab Biomechatronics; IMECE 2008, JRRD 2009, MIT PhD 2012).**
- The knee has two series-elastic actuators in an agonist-antagonist arrangement. In tests with four above-knee amputees, metabolic cost fell by an average of 5.8%. — [MIT thesis record](https://dspace.mit.edu/bitstream/1721.1/76513/2/823652940-MIT.pdf); [Media Lab publication page](https://www.media.mit.edu/publications/agonist-antagonist-active-knee-prosthesis-a-preliminary-study-in-level-ground-walking/) (search summary only)
- Patent text: the extension actuator is a motor, then a timing belt, then a precision ball screw. The flexion actuator uses a lead screw. The ball nut moves a spring cage, and the knee is driven by two steel cable drives, each looping around a joint pulley on one side of the knee. The cables tension independently, and a 1:2 ratio is given for this stage. — [US 8,500,823 "Powered artificial knee with agonist-antagonist actuation"](https://image-ppubs.uspto.gov/dirsearch-public/print/downloadPdf/8500823) (search summary only; the full PDF was not retrievable)

**Robonaut 2 hand, NASA JSC and General Motors (Bridgwater et al., ICRA 2012).**
- Each finger actuator is a motor, gearhead and ball screw assembly, plus a tendon, conduit, tension sensor and terminator, all housed in the forearm. — [Bridgwater et al., "The Robonaut 2 Hand – Designed To Do Work With Tools"](https://ntrs.nasa.gov/api/citations/20110023122/downloads/20110023122.pdf)
- Tendons are braided polymer. Spectra was dropped because its creep did not suit the ball screw's finite travel; the final tendon is a Vectran/Teflon hybrid weave. — [R2 hand paper](https://ntrs.nasa.gov/api/citations/20110023122/downloads/20110023122.pdf)
- For contrast, the R2 wrist uses a hollow-shaft motor driving a ball nut. The screw pushes a slider into a ball-and-socket linkage (no tendon), giving 27 kg force over 100 mm of travel. — [R2 hand paper](https://ntrs.nasa.gov/api/citations/20110023122/downloads/20110023122.pdf)
- Robonaut 1 hand (Lovchik & Diftler, about 1999): motors sit in the forearm, and finger motion comes from leadscrew assemblies driving the fingers through the wrist. — [NTRS 20000097045](https://ntrs.nasa.gov/api/citations/20000097045/downloads/20000097045.pdf) (search summary only)

**ARL Monopod, McGill (Gregorio, Ahmadi, Buehler, IEEE T-SMC-B 1997).** Paine & Sentis group this robot with Domo and Spring Flamingo as designs using a ball screw followed by a cable drive to a revolute joint. — [Paine & Sentis](https://sites.utexas.edu/hcrl/wp-content/uploads/sites/3888/2016/01/prismatic-sea-paine.pdf). I could not confirm the hip transmission from a primary source; the McGill pages do not describe it. — [McGill Monopod II page](https://library.cim.mcgill.ca/data/websites/mll/monopod2/monopod2.htm)

**UT-SEA, UT Austin HCRL (Paine & Sentis, about 2012, "A New Prismatic Series Elastic Actuator with Compact Size and High Performance"). Same reduction, different output: a pushrod, not a cable.**
- Drive: a BLDC motor, a 3:1 pulley stage on a low-backlash timing belt, then a ball screw in which the nut is the driven part, for about 175:1 overall. The joint is driven by a pushrod. A Vectran cable in the actuator only senses spring deflection. — [Paine & Sentis](https://sites.utexas.edu/hcrl/wp-content/uploads/sites/3888/2016/01/prismatic-sea-paine.pdf)
- They describe a pulley plus ball-screw reduction as typically above 90% efficient, impact resistant and backdrivable. They measured 77% forward mechanical efficiency for the whole actuator, with 110 W peak power and 15 rad/s top speed. — [Paine & Sentis](https://sites.utexas.edu/hcrl/wp-content/uploads/sites/3888/2016/01/prismatic-sea-paine.pdf)
- They cite Garrec's SCS as a "ball-nut-driven ... non-series-elastic cable-driven actuator" (their ref [34]). — [Paine & Sentis](https://sites.utexas.edu/hcrl/wp-content/uploads/sites/3888/2016/01/prismatic-sea-paine.pdf)

**Other mentions, not core:**
- Legged-robot SEAs that send power to the joint through "chain/cable" (Curran & Orin; Hutter et al., the ETH articulated leg) use rotary gearmotors and sprocket-to-sprocket or pulley-to-pulley drives, not a screw. — [Paine & Sentis, refs 5–6](https://sites.utexas.edu/hcrl/wp-content/uploads/sites/3888/2016/01/prismatic-sea-paine.pdf)
- Robinson's MIT thesis on series elasticity: cable transmissions have zero backlash and high efficiency because the cables run highly tensioned, but pulley size limits them to moderate ratios. — [Robinson, MIT thesis (ch. 1–3)](https://www.ftp.ai.mit.edu/people/dwrobin/pdf/thesis1-3.pdf)

### Inferences
- Spring Flamingo's drive, worked from its thesis numbers: knee motor-to-joint ratio = 1095 rad/s ÷ 6.9 rad/s ≈ 159:1, hip and ankle ≈ 111:1. That brackets Mike's quoted 119:1. Its motor, belt and ball screw sequence, with the output cable over an idler to a joint pulley, is topologically Mike's drive, with steel or polymer cable instead of roller chain and a series spring added.
- The CEA SCS inverts the screw: the nut turns and a hollow screw translates with the cable running through it. Mike's screw turns and the nut translates, with two chains outside the screw on either side. The CEA layout is built for backdrivability and haptic transparency, Mike's for a stiff, high-ratio position drive.
- In most research examples the motor stays off the moving link to cut distal inertia: in Spring Flamingo's body, Robonaut's forearm, and the prosthesis socket frame. ABLE embeds the SCS in the limb segment it drives, as Mike does.

### Gaps
- Spring Flamingo's cable material, and whether each joint uses a closed loop or a pair of antagonistic cables, were not stated in the sections read. Figures A-5 and A-6 show the routing but were not legible in text.
- I did not verify Hume's, Valkyrie's or Draco's drives (UT Austin and NASA); search results did not confirm any cable stage. The ARL Monopod hip transmission is unverified at the primary source.
- Toyota, KIST, IIT and Boston Dynamics: none of my searches turned up a screw-plus-wrapped-member joint drive from these groups. This is not proof of absence.
- I did not confirm the full MIT knee paper (JRRD 2009) text: the VA journal site's certificate had expired and the MIT Media Lab PDF host refused the connection.

## Q2. Which commercial actuators do this?

### Takeaway
The nearest industrial product family is the hydraulic piston-chain rotary actuator. Two pistons fixed to a roller chain pull it around an output sprocket, the same chain-and-sprocket topology as Mike's but driven by fluid, not a screw nut. In electromechanical form, the only commercial lineage found is CEA's screw-and-cable technology in Haption's Virtuose haptic arms. I found no catalog electric screw-plus-chain rotary actuator.

### Cited Findings
- Piston-chain rotary actuator: a large power piston has roller chain fixed to both sides. The chain wraps sprockets and ties to a smaller isolation piston. The net working area is the power piston minus the isolation piston. — [Power & Motion, "Book 2, Chapter 19: Rotary actuators"](https://www.powermotiontech.com/technologies/other-technologies/article/21882754/book-2-chapter-19-rotary-actuators)
- Another version: a circular chain held taut over two sprockets, with the output on one sprocket and an idler keeping tension. Two piston-shaped links ride on the chain, one larger than the other. Rotation reaches five full turns and torque up to 23,500 lb-in. The design is limited by chain and sprocket strength and by its bulk at very high torque. — [Hydraulics & Pneumatics, "Rotary Actuators: Turnabout is Fair Play"](https://www.hydraulicspneumatics.com/200/TechZone/FluidPowerAcces/Article/False/9517/TechZone-FluidPowerAcces); [Machine Design, Rotary actuators](https://www.machinedesign.com/basics-design/rotary-actuators) (search summary only)
- A rotary valve actuator patent says silent chain can replace roller chain in this mechanism, for slightly more precision at slightly higher cost. — [US 7,607,640](https://patents.google.com/patent/US7607640) (search summary only). Early patent for the type: [US 3,267,816 "Rotary actuator device"](https://patents.google.com/patent/US3267816).
- Rack-and-pinion fluid actuators are the common alternative: up to 5 rotations; with air up to about 200,000 in-lb; typically 2,500–3,500 in-lb for the double-rack, pressurised-pinion type. — [Power & Motion, Ch. 19](https://www.powermotiontech.com/technologies/other-technologies/article/21882754/book-2-chapter-19-rotary-actuators)
- Haption (founded 2001 out of CEA force-feedback research) sells the Virtuose 6D line. A CEA/Haption presentation reportedly shows the circa-2001 Virtuose 6D with one screw-cable (SCS) actuator per axis plus a gripper actuator (search summary only; garbled source). — [Haption presentation, LIRMM](https://www.lirmm.fr/uee07/presentations/forum/HaptionLIRMM.pdf); [CEA success story: Haption](https://www.cea.fr/cea-tech/english/Pages/success-stories/haption.aspx)
- Surgical-robot patents describe lead-screw carriages that pull cable pairs wrapped around wrist pulleys through a sterile-drape interface. — [US 10,932,878 "Surgical drape for transferring drive"](https://image-ppubs.uspto.gov/dirsearch-public/print/downloadPdf/10932878) (search summary only; assignee and product not verified)

### Inferences
- The piston-chain actuator is the industrial proof that a linearly pulled roller chain around a sprocket works as a constant-torque rotary drive over a limited angle. The torque limit is set by chain and sprocket strength.
- Haption's current datasheets give no transmission details, so it is unconfirmed whether today's Virtuose models still use SCS.

### Gaps
- I found no named current manufacturer or model of piston-chain rotary actuators. The trade articles do not name makers, and the catalogs searched (HKS, Parker, Moog Flo-Tork) showed only rack-and-pinion, vane or helical types in the excerpts.
- No commercial electric ball-screw-plus-cable (or belt or chain) rotary actuator was found as a catalog item.
- Backlash and efficiency figures for piston-chain actuators were not found.

## Q3. Is there anything that uses ROLLER CHAIN specifically, pulled by a SCREW NUT, to rotate a robot joint?

### Takeaway
No research robot, commercial robot or catalog actuator found does this. The research examples use steel cable (Spring Flamingo, the MIT knee), polymer tendon (Robonaut 2, UT-SEA's sensing cable), a push-pull cable (CEA SCS) or a timing belt (a patent). Roller chain pulled linearly around a sprocket appears only in hydraulic piston-chain rotary actuators, and in legged robots where a rotary gearmotor drives the chain sprocket to sprocket.

### Cited Findings
- Piston-chain hydraulic rotary actuators: a piston pulls a roller chain around the output sprocket. — [Power & Motion Ch. 19](https://www.powermotiontech.com/technologies/other-technologies/article/21882754/book-2-chapter-19-rotary-actuators)
- Legged-robot SEAs that move power to the joint by "chain/cable" are driven by rotary gearmotors, not screws. — [Paine & Sentis](https://sites.utexas.edu/hcrl/wp-content/uploads/sites/3888/2016/01/prismatic-sea-paine.pdf)
- Near-miss patent: in the "Speed reducer", nuts are fastened to each link of a roller chain and a recirculating-ball screw drives the chain directly. That screw meshes with the chain itself, so it is not a nut pulling a chain. — [US 9,234,570](https://image-ppubs.uspto.gov/dirsearch-public/print/downloadPdf/9234570) (search summary only)
- Patents seen in passing, all covered by other researchers: a ball nut plus wire robot joint drive ([US 8,714,045](https://image-ppubs.uspto.gov/dirsearch-public/print/downloadPdf/8714045)); a ball screw with tensile member and guide pulleys driving an exoskeleton joint pulley ([US 10,702,441](https://image-ppubs.uspto.gov/dirsearch-public/print/downloadPdf/10702441)); a wearable knee brace in which a ball nut drives a belt around the knee drum, or a cable in some versions ("Hybrid terrain-adaptive lower-extremity systems", e.g. [US 8,419,804](https://image-ppubs.uspto.gov/dirsearch-public/print/downloadPdf/8419804)). All are search summary only; the exact patent in that family holding the belt text was not pinned down.

### Inferences
- Using standard roller chain, with two strands flanking the screw and a closed loop through an idler, looks unusual in the research and industrial literature searched. Its likely draws are steel strength in a small package, off-the-shelf sprockets and no crimped cable ends (crimp failure was Domo's reported cable weak point).
- A chain wrapped on a sprocket has polygonal (chordal) action. Its effective radius swings between R·cos(180°/N) and R, which for N = 25 is about 0.8%, so expect a small periodic torque and ratio ripple per tooth pitch. A cable on a smooth drum has none.

### Gaps
- Patents and hobby or open-source builds were out of scope; other researchers cover them. A roller-chain version may exist there.

## Q4. How do the closest examples compare (tension member, screw, travel or angle, motor location, backlash, efficiency)?

### Takeaway
Spring Flamingo matches the drivetrain topology and the overall ratio; CEA SCS matches the idea of screw and tension member embedded in the link. Mike's design differs from all of them in four ways: roller chain instead of cable, two strands flanking a rotating screw, the motor riding on the driven link's parent, and no series spring or force sensing.

### Cited Findings (data per example; sources as in Q1–Q2)

| Example (year) | Tension member | Screw | Joint pulley | Angle / travel | Motor location | Efficiency / backlash notes | Source |
|---|---|---|---|---|---|---|---|
| **Mike's arm (2026)** | 2 × #25 roller chain, closed loop via idler | SFU1204 ball screw, 4 mm lead; screw rotates, nut translates; 1:1 belt from NEMA 17 | 25T sprocket (PD ≈ 50.7 mm) | ≈64 mm → ≈145° | On the link | Steel throughout; no published data | brief |
| **Spring Flamingo (MIT Leg Lab, ~1996–2000)** | Cable (material not stated) from actuator output over an idler to the joint pulley | Ball screw, 0.125 in (3.175 mm) lead; 2.5:1 timing belt; screw driven | 44 mm (hip/ankle), 64 mm (knee) | Hip ≈±45°, knee ≈90°, ankle ≈+15/−45° | In the body; cables routed through the hip | Series springs; 15 Hz force bandwidth; ratio ≈111:1 / 159:1 (computed) | [Pratt 2000](https://www.ftp.ai.mit.edu/pub/users/jpratt/phd_thesis.pdf) |
| **Domo (MIT CSAIL, 2004)** | Cable | Precision ball screw, motor-driven | n/a | Limited by screw travel (authors) | Arm actuators | Series springs, 300 lb/in; crimp breakage noted | [Domo](https://groups.csail.mit.edu/lbr/mars/pubs/humanoids04.pdf) |
| **CEA SCS / ABLE (2000 patent; 2008 robot)** | Push-pull cable through a hollow screw, closed loop | Ball screw; nut rotates, screw translates (no rail) | n/a | One axis listed at 110° | Embedded in the limb segment, parallel to the cable | Low friction threshold, high backdrivability (claimed); 18/13 Nm continuous | [IROS 2008](https://infoscience.epfl.ch/record/181352/files/04651014-1.pdf?version=1) |
| **MIT agonist-antagonist knee (2008–2012)** | 2 steel cables looping joint pulleys on each side of the knee | Ball screw (extension) + lead screw (flexion), belt-driven | n/a | n/a | On the prosthesis frame | 1:2 cable stage (patent); series springs | [US 8,500,823](https://image-ppubs.uspto.gov/dirsearch-public/print/downloadPdf/8500823) |
| **Robonaut 2 hand (2012)** | Braided polymer tendon (Vectran/Teflon) in a conduit | Ball screw assembly per tendon | Finger joints | Finite ball-screw travel; creep mattered | Forearm (remote) | Tension sensor per tendon | [R2 hand](https://ntrs.nasa.gov/api/citations/20110023122/downloads/20110023122.pdf) |
| **UT-SEA (2012), for contrast** | None (pushrod); Vectran only for sensing | Ball screw, nut driven; 3:1 belt; ≈175:1 | Pushrod crank | n/a | On the leg | 77% measured forward efficiency | [Paine & Sentis](https://sites.utexas.edu/hcrl/wp-content/uploads/sites/3888/2016/01/prismatic-sea-paine.pdf) |
| **Piston-chain hydraulic actuator** | Roller chain (or silent chain) | Hydraulic pistons on the chain | Output sprocket + idler | Up to 5 turns | n/a | Constant torque over the stroke; limited by chain strength | [Power & Motion](https://www.powermotiontech.com/technologies/other-technologies/article/21882754/book-2-chapter-19-rotary-actuators) |

- Cable transmissions: zero backlash and high efficiency when kept highly tensioned, but only moderate ratios. — [Robinson thesis](https://www.ftp.ai.mit.edu/people/dwrobin/pdf/thesis1-3.pdf)
- Push-pull cable drives in exoskeletons can reach up to 96% efficiency if properly implemented; efficiency depends on wrap angle, cable and housing. — [Frontiers in Robotics and AI 2018](https://www.frontiersin.org/journals/robotics-and-ai/articles/10.3389/frobt.2018.00105/pdf) (search summary only)

### Inferences
- **Ratio check on Mike's numbers.** A #25 25T sprocket has pitch diameter 6.35 mm / sin(7.2°) ≈ 50.7 mm (radius ≈ 25.3 mm). The quoted 64 mm → 145° is consistent with that: 64 / 25.3 = 2.53 rad = 145°. One joint revolution is 2π × 25.3 ≈ 159 mm of nut travel, which is ≈ 39.8 turns of a 4 mm-lead screw. With a 1:1 belt the motor-to-joint ratio is therefore ≈ 40:1, not 119:1. 119:1 is almost exactly 3 × 39.8, which would match a 3:1 belt (or a ≈1.33 mm lead). The brief's "1:1 belt" and "≈119:1" cannot both hold, and this should be checked against the hardware.
- Mike's drive has the same topology as Spring Flamingo's: motor, belt, ball screw, nut-carried output, tension member over an idler, joint pulley. Spring Flamingo had 3.175 mm lead, a 2.5:1 belt and 44–64 mm pulleys, giving ≈111–159:1. Mike's 4 mm lead and ≈51 mm sprocket give ≈40:1 at 1:1 (≈119:1 at 3:1).
- Backlash: in all the cable examples, backlash is removed by pretension in a closed loop or antagonistic pair. Mike's chain loop through an idler can be preloaded the same way, but roller chain adds pin and bushing clearance, wear stretch and the ≈0.8% chordal ripple worked out above. The ball screw's axial play (SFU single nut, typically not preloaded) adds directly at the nut.
- Efficiency: a ball screw plus pulley stage is quoted above 90% (Paine & Sentis). One full measured actuator with belt, screw and pushrod reached 77%. No chain-wrap efficiency data from robotics was found; chain wrap losses come from pin articulation at the sprocket entry and exit.
- Travel: every screw-plus-wrap drive is limited by screw stroke ÷ pulley radius. Mike's ≈145° sits within the range of the research joints (≈90–110° in Spring Flamingo and ABLE) and far below the piston-chain actuators' multi-turn capability.

### Gaps
- No measured backlash figures were found for Spring Flamingo, Domo, ABLE or the MIT knee.
- No published efficiency for roller chain used as a partial-wrap linear-to-rotary element in a robot.
- Spring Flamingo's cable material and diameter, and ABLE's screw size and lead, were not found in the sections read.
