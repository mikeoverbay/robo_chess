# Patents: screw-driven nut pulling a chain / cable / belt around a joint sprocket or pulley

*Quick prior-art scan, not a legal opinion or freedom-to-operate search. Statuses come from Google Patents' "legal status" field, which Google itself labels an assumption. Google Patents started returning HTTP 503 partway through, so some patents below were checked only through search snippets, as noted.*

Reference mechanism (Mike's arm, 2026): SFU1204 ball screw along the arm link, NEMA 17 on the link driving it via a 1:1 belt, ball nut fixed to two #25 roller chains (one per side), each chain: nut -> 25T sprocket fixed to the next joint -> small idler -> back to nut. About 64 mm of travel = about 145 deg, about 119:1.

## Q1. Patents that claim or show a screw-driven nut pulling a chain (especially roller chain) around a sprocket to rotate a joint or shaft

### Takeaway
I found no patent showing a screw-driven nut pulling a roller chain around a sprocket to rotate a joint. The closest chain-plus-sprocket hit is a Suncore/Emcore solar tracker (linear actuator, chain anchored to the moving coupling, sprocket on the torque tube, plus a smaller second sprocket), but the actuator type is not stated and nothing rides on a moving link. Every other chain-plus-sprocket hit is driven by hydraulic or pneumatic rams or pressure tubes, not a screw.

### Cited Findings
- **US8536504B2, "Terrestrial solar tracking photovoltaic array with chain drive"** (Suncore Photovoltaics; assigned from Emcore Solar Power). Priority 2008-10-24, filed 2012-01-31, granted 2013-09-17. Status: "Expired - Fee Related" (Google's anticipated expiry was 2028-10-24). Linear actuator 571 has a movable shaft/piston 580. A coupling on the shaft rides on wheels in parallel tracks. Two chain members are anchored by bolts to that coupling. An upper sprocket at the top of the post is fixed to the torque-tube coupling plates. A smaller lower sprocket sits on a bracket. The support rotates "in excess of 180°". The fetched text does not say whether the actuator is a screw or hydraulic type — [Google Patents](https://patents.google.com/patent/US8536504B2/en); [Justia](https://patents.justia.com/patent/8536504)
- **WO2010002043A1, "Robot having rotatable arm"** (Seoul National University / Daewoo Shipbuilding; priority 2008-06-30; status "Ceased"). It uses chains around sprockets on arm bodies, but only for linear lift and extension: an elevating plate on one chain run and the extensible arm on the other. Rotation is by a motor and reducer, and the drive is rack and pinion, not a screw. Not a match — [Google Patents](https://patents.google.com/patent/WO2010002043A1/en)
- **US7258521B2, "Chain-driven robotic arm"** (ScriptPro LLC; filed 2005; expired). A motor-driven sprocket pushes a stiff chain to telescope an arm. No screw or nut, no pivot joint. Not a match — [Google Patents](https://patents.google.com/patent/US7258521B2/en)
- **US8322249B2, "Robot arm assembly."** A chain inside the lower arm between two sprockets raises and lowers the upper arm, with the sprocket driven through a gearbox. No screw/nut drive in the snippet — [Google Patents](https://patents.google.com/patent/US8322249B2/en)
- A Google Patents Boolean search (nut NEAR/8 chain, sprocket, screw, CPC B25J9/104 "cables, chains or ribbons") returned 70 hits. None in the visible top 42 was a screw-nut-chain-sprocket joint. They were mostly warehouse robots, lifts, grippers and exoskeletons — [Google Patents query](https://patents.google.com/xhr/query?url=q%3D(nut+NEAR%2F8+chain)+AND+sprocket+AND+screw%26cpc%3DB25J9%2F104%26num%3D50&exp=)

### Inferences
- In the patent record, a roller chain wrapped on a sprocket and pulled linearly is almost always driven by fluid rams (see Q3). A ball or lead screw pulling a chain loop seems rare, or is buried in Chinese/Japanese utility models that this scan did not reach.
- The Suncore tracker is the closest chain-and-sprocket analogue outside robotics. It shares "linear actuator + chain anchored to the moving carriage + sprocket on the rotating shaft + second, smaller sprocket", but the drive is a fixed post, not a joint on a moving link.

### Gaps
- I could not confirm the Suncore actuator type (screw or hydraulic) or read its claims. Google Patents truncated the page, and the later fetch hit a 503.
- Full-text Boolean searches on Google Patents were noisy, and the service rate-limited (503) before narrower queries could run. Espacenet, FreePatentsOnline and Justia refused automated fetches (403). Chinese (丝杠/链条/链轮) and Japanese (ボールねじ/チェーン/スプロケット) utility models are therefore under-covered. A manual search there is the most likely place to find a closer chain hit.

## Q2. The same with cable / belt / band (screw + nut + wrapped cable around a joint pulley) in robot arms, legs, exoskeletons or prosthetics

### Takeaway
This is well covered. The cable version of Mike's layout is patented several times since 2003, by CEA, Chongqing University, Samsung, MIT, GM/NASA and Ekso. The closest is **Samsung US8635929B2**: motor -> belt -> ball screw -> movable member -> closed steel-cable loop over an idle pulley and an output pulley that drives the joint. It is the same loop topology as Mike's nut -> sprocket -> idler -> nut, and Google lists it as still active.

### Cited Findings
- **US8635929B2, "Robot joint driving apparatus, robot having the same and cable linkage method of robot joint driving apparatus"** (Samsung Electronics). Priority 2010-01-14 (KR), filed 2011-01-10, granted 2014-01-28 (published as US20110167945A1).
  - Status: "Active". Google's adjusted expiry is 2031-11-24, and a 12th-year maintenance fee was paid in July 2025.
  - Mechanism: a reversible motor drives a ball screw through a driving belt. A guided movable member translates on the screw. A steel cable closed loop (ends joined by a sleeve) is fixed to both sides of the movable member and runs around an unpowered idle pulley and an output pulley that "directly drives the joint unit". It runs as four parallel lines for torsional stiffness, with cable guide brackets and fixing sleeves seated in grooves to prevent slip.
  - Claim 1 recites a movable member moving "rectilinearly through driving force transmitted from the driving motor" and lines of the cable assembly "connected at least once in parallel among the idle pulley, the movable member, and the output pulley". The ball screw appears only in dependent claims 6–7.
  - It cites CEA US7389974B2 and Mako, KAERI and Rosheim cable patents. It is cited by Ekso US10702441B2 and Boston Dynamics US12123481B2.
  - Source: [Google Patents](https://patents.google.com/patent/US8635929B2/en)
- **CN1272147C, "Robot rope-driven joint" (机器人绳驱动关节)** (Chongqing University; inventor Liang Xichang). Filed 2003-03-13, granted 2006-08-30. Expired for non-payment (lapse recorded 2008-05-14). Servo motor -> toothed belt -> rotating nut -> lead screw translates (a guide pin stops rotation). A steel wire rope is attached to both ends of the screw. It wraps 270° around the output sheave (fixed to the output joint, pinned through a hole), then 270° more, then runs to a follower sheave and back to the other end of the screw. The joint turns about 160°. Stated aims: simple, large output torque, compact — [Google Patents](https://patents.google.com/patent/CN1272147C/en)
- **US7389974B2, "Screw, nut and cable transmission"** (CEA; inventor Philippe Garrec). Priority 2003-03-14 (FR), granted 2008-06-24. "Expired - Fee Related" (lapsed 2020-07-27). A hollow-rotor motor turns a nut, and the screw translates. The cable is attached mid-screw near the nut. Two lines run to double pulleys (small/large diameter for travel amplification), then to the joint pulley on the driven lever. One pulley axis is movable to tension the loop. A guide pulley is an option. Cable only, chain not mentioned. Related CEA family: US7073406B2 (screw transmission with cable attached to screw) and US7574939B2 (articulated-arm segment with screw and nut transmission) — [Google Patents](https://patents.google.com/patent/US7389974B2/en)
- **US10702441B2, "Ball screw and tensile member exoskeleton joint actuation device"** (Ekso Bionics; inventors Aaron Julin, Mike Hughes; SOCOM contract). Priority 2016-05-04, filed 2017-04-27, granted 2020-07-07; EP counterpart EP3243606B1.
  - Status: "Expired - Fee Related", despite an adjusted expiry of 2038.
  - Mechanism: a motor (in line, or parallel via "a gear, wheel, belt, or other system") drives a ball screw. The ball nut carries cord reactors and a load cell. Opposed cords, fixed at both ends to a joint pulley body, run back over guide pulleys so their runs lie parallel to the screw. This balances the nut and removes the need for side-load guide rails. Cords or metal cables are used. Chain is not named as a tensile member.
  - Sources: [Google Patents US](https://patents.google.com/patent/US10702441B2/en); [Google Patents EP](https://patents.google.com/patent/EP3243606B1/en)
- **US8500823 / US9149370, "Powered artificial knee with agonist-antagonist actuation"** (MIT; inventors Herr, Martinez-Villalpando, Weber). Priority 2009-01-30; EP2398425. Two series-elastic actuators in parallel. Per a search-tool summary of the USPTO PDF, each transmission is a timing pulley and belt driving a ball screw, and "two steel cables, each looping around its corresponding joint pulley located on each side of the knee". I could not open the full text to verify. Status not checked — [USPTO PDF](https://image-ppubs.uspto.gov/dirsearch-public/print/downloadPdf/8500823); [EPO front page](https://data.epo.org/publication-server/rest/v1.2/patents/EP2398425NWA1/document.html); [MIT TLO](https://tlo.mit.edu/industry-entrepreneurs/available-technologies/biomimetic-robotic-leg-agonist-antagonist-actuation)
- **US8919842B2, "Robot arm with tendon connector plate and linear actuator"** (GM/NASA Robonaut lineage), and **US8467903** (humanoid finger actuator). A motor-driven ball screw translates a ball nut that carries a tendon hook. These are one-way tendon pulls to fingers, not a closed loop around a joint pulley — [Google Patents](https://patents.google.com/patent/US8919842B2/en); [US8467903 PDF](https://patentimages.storage.googleapis.com/21/dc/f7/32af9afaaee3a0/US8467903.pdf)
- **US10561563B2, "Optimal design of a lower limb exoskeleton or orthosis."** Per the search snippet, it has a winch or ball-screw actuator in the links and a large pulley applying knee extension moment. Assignee and details not verified — [Google Patents](https://patents.google.com/patent/US10561563B2/en)
- **Boston Dynamics "Screw actuator for a legged robot"** (US10253855, US11131368, US11754155, US12123481; EP3554771B1). A screw shaft and nut lie inside the upper leg member, driven by a motor at the hip. A carrier on the nut drives a linkage that rotates the lower leg at the knee. This is "screw along the link", but with a linkage instead of a wrapped flexible member. US10253855 was granted in 2019 and reexamined in 2025, with claims 1–4 confirmed and others cancelled — [uspto.report](https://uspto.report/patent/grant/10253855); [EP3554771B1](https://patents.google.com/patent/EP3554771B1/en); [USPTO Gazette reexam](https://patentsgazette.uspto.gov/week28/OG/html/1536-3/US10253855-20250715.html)
- **US8516913B2, "Actuator"** (T. J. Jones; now Moog). Rotary output for robot-arm control wires, made with a differential toothed belt loop. It has **no** screw. "Belt" is defined to include chain or cable — [Google Patents](https://patents.google.com/patent/US8516913B2/en)

### Inferences
- Samsung US8635929B2 has the same topology as Mike's chain loop, cable for chain: a belt-driven ball screw, a movable member on the screw, a closed flexible loop from both sides of the member, around an idle pulley and an output pulley that drives the joint. Its claim 1 is built on a "cable assembly" with lines "connected at least once in parallel". Whether a pair of roller chains reads on that is a legal question this scan cannot answer.
- CN1272147C (2003) already shows "belt -> nut/screw -> rope loop around joint sheave and follower sheave, about 160° of joint travel", so the general cable-loop idea predates Samsung.
- Ekso's routing (flexible runs brought back parallel to the screw so the nut needs no side guide) resembles Mike's use of two chains, one on each side of the screw. Ekso uses cords.

### Gaps
- MIT knee cable routing is unverified from the primary text, and the status of the MIT family is not checked.
- Samsung's related family (US8714045B2, US8459139B2, US8425363B2, US20090308188A1) was listed as "similar" but not opened, and Hiwin "Pulley actuator" US7810793B1 (cites CEA) could not be opened (503/403). Their mechanisms are unknown.
- None of these documents states whether the screw and motor sit on the moving link (as in Mike's arm) or on the fixed side. Only Boston Dynamics clearly puts the screw inside the moving leg member.

## Q3. "Linear-to-rotary" chain actuators (cylinder or screw pulling a chain around a sprocket) and patents covering them

### Takeaway
Chain rotary actuators are an old, established category, mostly fluid-driven: double-acting pistons or rams pulling a chain over a sprocket or drum, used in industrial actuators, valve actuators and backhoe swing drives from the 1960s on. Screw-driven versions did not show up. Screw-driven quarter-turn valve actuators exist, but they use cranks or cams, not chains.

### Cited Findings
- **US3267816A, "Rotary actuator device"** (Graham, 1960s): "a chain sprocket is rotatably mounted upon an output shaft", with a chain interconnecting adjacent ends of double-acting pistons. Fluid-driven — [Google Patents](https://patents.google.com/patent/US3267816A/en)
- **FR2417009A1** (1979, Descottes), "Hydraulic or pneumatic motor": a double-acting ram coupled by a pin to parallel endless chains running around sprockets. Cited by the Hiflex patent below — [cited on US6860189B2 page](https://patents.google.com/patent/US6860189B2/en)
- **US6860189B2 / US20040129132A1, "Rotary actuator with cartridge and chain or cable"** (Hiflex Technologies; inventor Jose Perez). Priority 2001-04-06, granted 2005-03-01. Lapsed 2013 for non-payment. Two pressurized elastomer tubes pull a chain over a sprocket on the output shaft. A steel cable over a pulley (fixed to the pulley, under 180° of travel) is an alternative. No screw option. It claims to avoid the side loads of rack-and-pinion and scotch-yoke actuators — [Google Patents](https://patents.google.com/patent/US6860189B2/en)
- **US7607640B2, "Rotary valve actuator"**: its background says a loop of chain tied to double-ended cylinder rods and driving a sprocket is a known linear-to-rotary method, "used only in special applications" — [Google Patents](https://patents.google.com/patent/US7607640)
- **US9702442B2, "Rotary actuators with chain operated cams"** (Emerson Process Management Valve Automation; inventor Harry Hobert; granted 2017-07-11). Two chains wrap a cam's outer edge in opposite directions and are driven by a power-module rod and a spring module. Chain on a cam, not a sprocket — [Google Patents](https://patents.google.com/patent/US9702442)
- **US6000675A** (Fisher Controls; tension-spring return valve actuator): bladder-driven canister, linear motion converted via chain to rotary lever. **US4225110A** (Baker): flexible segmented chain links two push rods to a rotor — [cited on US6860189B2 page](https://patents.google.com/patent/US6860189B2/en); [US4225110 PDF](https://image-ppubs.uspto.gov/dirsearch-public/print/downloadPdf/4225110)
- **Backhoe and boom swing drives:**
  - **US4003484** (Jones et al., "Swing mechanism for implement boom"): roller chains wrap a pair of stacked sprockets on the swing shaft in opposite directions.
  - **US4078680** (Pilch): two rams each pull a chain wrapped on a common drum, one per direction.
  - **US4904149** (Pilch, Ware Machine Works, 1990): a single double-acting actuator drives the chain, with automatic chain-wear compensation.
  - **US3202000A** (International Harvester, "Slewing mechanism"): a hydraulic cylinder pulls a cable wrapped on a helical pulley, for 360°.
  - Sources: [US4003484](https://image-ppubs.uspto.gov/dirsearch-public/print/downloadPdf/4003484); [US4078680](https://patents.google.com/patent/US4078680); [US4904149](https://image-ppubs.uspto.gov/dirsearch-public/print/downloadPdf/4904149); [US3202000A](https://patents.google.com/patent/US3202000A/en)
- **Screw-driven quarter-turn valve actuators without chains:** US5085401 (motor-driven Acme screw, traveling block to 90° output). US4325535 (ball screw back-driven nut rotates a valve stem) — [US5085401](https://patents.google.com/patent/US5085401); [US4325535](https://patents.google.com/patent/US4325535A/en)
- **Antenna and solar drives:**
  - US3952984 turns an antenna boom with an endless sprocket chain. US2654031 uses a drum, cable and pulleys for azimuth and elevation. Neither driver is shown as a screw in the snippets.
  - Solar trackers mostly use linear actuators on torque arms (US11264942, US6058930) rather than chain wraps. Suncore US8536504B2 (Q1) is the chain exception.
  - Sources: [US3952984](https://patents.google.com/patent/US3952984); [US2654031](https://patents.google.com/patent/US2654031); [US11264942](https://image-ppubs.uspto.gov/dirsearch-public/print/downloadPdf/11264942); [US6058930](https://patents.google.com/patent/US6058930)
- **Furniture:** lead-screw recliners and lift chairs (US5280999, US5482350) use nut-and-cam or nut-and-link, with no chain-and-sprocket variant found — [US5280999](https://patents.google.com/patent/US5280999); [US5482350](https://image-ppubs.uspto.gov/dirsearch-public/print/downloadPdf/5482350)

### Inferences
- "A pulled chain wrapped on a sprocket gives rotation" is old and public-domain art (1960s–1990 patents, all long expired). A screw in place of the ram is the main variable in Mike's design, and this scan found no patent claiming that swap for a chain.

### Gaps
- I did not open US3267816A, FR2417009A1 or US4003484 in full, so the exact chain routing (idlers, whether the chain is a closed loop) is from snippets and citation summaries.
- No hospital bed, telescope or vehicle patent with screw + chain + sprocket was found in the time available. That is absence in this scan, not proof of absence.

## Q4. Is the specific combination (screw along the link + roller chain loop around the joint sprocket + idler, motor on the link) shown anywhere? If not, what is closest and what is missing?

### Takeaway
I did not find this combination in any patent. Three documents together cover nearly every element: Samsung US8635929B2 (loop topology, cable), Boston Dynamics' screw actuator (screw inside the moving link, linkage) and Suncore US8536504B2 (linear actuator + chain + sprocket + second sprocket, fixed post). The element not found anywhere is a **roller chain** on a **screw-driven nut** forming a nut–sprocket–idler–nut loop, with the motor and screw carried on the link.

### Cited Findings
- **Closest overall: Samsung US8635929B2.** It shows a belt-driven ball screw, a movable member, a closed flexible loop from both sides of the member, an idle pulley, and an output pulley that directly drives the joint. Differences from Mike's arm:
  - steel cable (four parallel lines, fixed with sleeves in pulley grooves) instead of two roller chains on toothed sprockets;
  - the location of the motor and screw relative to the moving link is not stated.
  - Source: [Google Patents](https://patents.google.com/patent/US8635929B2/en)
- **CN1272147C** (Chongqing, 2003): belt -> rotating nut -> translating screw -> rope loop around output sheave and follower sheave. Differences: rope not chain; the screw translates (nut rotates) instead of a fixed screw with a running nut; and the rope is pinned through the sheave rather than meshing on teeth — [Google Patents](https://patents.google.com/patent/CN1272147C/en)
- **Ekso US10702441B2**: ball nut with cords returned parallel to the screw via guide pulleys, so no nut side-guide is needed, motor optionally belt-coupled in parallel. Difference: cords to a joint pulley body, not chain on a sprocket — [Google Patents](https://patents.google.com/patent/US10702441B2/en)
- **Boston Dynamics US10253855 family**: screw and nut inside the moving upper-leg link, driven from the hip. Difference: nut drives a linkage, not a wrapped chain or cable — [uspto.report](https://uspto.report/patent/grant/10253855)
- **Suncore US8536504B2**: linear actuator with chain anchored to its moving coupling, sprocket on the rotating tube, smaller second sprocket, over 180°. Differences: fixed post, not an arm link; actuator type unconfirmed; solar-tracker field — [Google Patents](https://patents.google.com/patent/US8536504B2/en)

### Inferences
- Mike's combination reads as Samsung/CN1272147C topology with roller chain swapped in for cable, on a link-mounted drive like Boston Dynamics'. The swap of cable for roller chain on a sprocket is the main structural difference from the closest prior art. Roller chain on sprockets is generic, well-known art from the fluid-ram actuators in Q3. This is an engineering observation, not a patentability or infringement opinion.

### Gaps
- No full claim reading was done on any patent. Whether "cable assembly" in Samsung's claims, or similar terms elsewhere, would cover a roller chain is a legal question outside this scan.
- Chinese/Japanese utility models were not searched in depth. A CN utility model with 丝杠 + 链条 + 链轮 driving a joint would not be surprising and was not ruled out.

## Q5. Are any of the closest patents recently active (filed in the last ~20 years)? Expiry notes

### Takeaway
Of the closest patents, Google lists only **Samsung US8635929B2** as clearly in force (estimated expiry around 2031-11, fees paid 2025). The Boston Dynamics screw-actuator family (grants 2019–2024, reexamined 2025) is recent but its mechanism differs (linkage). Ekso, CEA, Chongqing, Hiflex and Suncore are all listed as lapsed or expired for unpaid fees. The fluid-ram chain actuators are decades past expiry.

### Cited Findings
- **US8635929B2** (Samsung): filed 2011-01-10, "Active", adjusted expiry 2031-11-24, 12th-year maintenance fee paid July 2025 — [Google Patents](https://patents.google.com/patent/US8635929B2/en)
- **US10702441B2** (Ekso): filed 2017. Google says "Expired - Fee Related" (adjusted expiry would have been 2038). The status of EP3243606B1 was not checked — [Google Patents](https://patents.google.com/patent/US10702441B2/en)
- **Boston Dynamics screw actuator family**: US10253855 (granted 2019; reexam certificate C1 in 2025, claims 1–4 confirmed), US11131368 (reexam C1 2025), US11754155, US12123481 (2024). Maintenance status not checked — [USPTO Gazette US10253855 C1](https://patentsgazette.uspto.gov/week28/OG/html/1536-3/US10253855-20250715.html); [USPTO Gazette US11131368 C1](https://patentsgazette.uspto.gov/week05/OG/html/1531-1/US11131368-20250204.html); [US12123481](https://image-ppubs.uspto.gov/dirsearch-public/print/downloadPdf/12123481)
- **US7389974B2** (CEA): lapsed 2020-07-27 — [Google Patents](https://patents.google.com/patent/US7389974B2/en)
- **CN1272147C** (Chongqing): lapsed 2008 — [Google Patents](https://patents.google.com/patent/CN1272147C/en)
- **US6860189B2** (Hiflex): lapsed 2013 — [Google Patents](https://patents.google.com/patent/US6860189B2/en)
- **US8536504B2** (Suncore): "Expired - Fee Related" — [Google Patents](https://patents.google.com/patent/US8536504B2/en)
- **US9702442B2** (Emerson, granted 2017): chain-on-cam valve actuator, a different field. Status not checked — [Google Patents](https://patents.google.com/patent/US9702442)

### Inferences
- US utility patents run about 20 years from filing if fees are paid. On that basis the MIT knee family (2009 priority) would end around 2030 at the latest, and Samsung around 2031 (Google's adjusted date). Pre-2006 filings are expired regardless of fees.

### Gaps
- I did not check maintenance status for MIT US8500823/US9149370, the Boston Dynamics family, Emerson US9702442B2, or any EP or CN counterparts. Google's status labels are its own assumption, not official register data.
