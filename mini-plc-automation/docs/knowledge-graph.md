# Knowledge Graph — mini-plc-automation

## L1: Definitions (Complete)
- PLC operational modes (RUN, STOP, PROGRAM, DEBUG, FAULT, MAINTENANCE)
- Scan cycle phases (INPUT_SCAN, PROGRAM_EXEC, OUTPUT_SCAN, HOUSEKEEPING)
- IEC 61131-3 data types (BOOL, BYTE, WORD, DWORD, INT, REAL, TIME, etc.)
- I/O addressing model (%I, %Q, %AI, %AQ, %M, %MW, %T, %C, %S)
- Timer types (TON, TOF, TP, RTO)
- Counter types (CTU, CTD, CTUD)
- Ladder Diagram elements (contacts NO/NC/POS/NEG, coils, FBs)
- FBD block types (AND, OR, XOR, ADD, SUB, MUL, DIV, PID, etc.)
- ST language constructs (IF, CASE, FOR, WHILE, REPEAT)
- PLC system state (plc_system_t)

## L2: Core Concepts (Complete)
- Cyclic scan execution model
- I/O image table (double-buffering)
- Timer operation (accumulation, preset comparison)
- Counter operation (edge detection, up/down)
- Ladder rung evaluation (power flow)
- FBD network execution (DAG topological sort)
- Structured Text interpretation (AST walk)
- Digital I/O access patterns
- Analog scaling and filtering (4-20mA, 0-10V)

## L3: Mathematical Structures (Complete)
- Boolean algebra for ladder logic (AND series, OR parallel, NOT NC)
- SR/RS flip-flop state equations
- Edge detection as discrete-time Boolean differentiation
- SFC state-transition model
- DAG representation of FBD networks
- Adjacency matrix for ladder rung analysis
- Topological sorting (Kahn's algorithm)
- Short-circuit Boolean evaluation semantics

## L4: Fundamental Laws (Complete)
- Liu & Layland RMS schedulability bound (1973)
- Nyquist-Shannon sampling theorem applied to PLC I/O
- Scan cycle timing model: T_scan = T_input + T_program + T_output + T_housekeeping
- Watchdog timeout safety property
- Callendar-Van Dusen equation for RTD linearization
- ITS-90 thermocouple inverse polynomial

## L5: Algorithms/Methods (Complete)
- RMS schedulability test
- PID controller (parallel form, backward Euler, anti-windup)
- Ziegler-Nichols open-loop tuning (FOPDT)
- Ziegler-Nichols closed-loop tuning (Astrom-Hagglund relay method)
- First-order IIR low-pass filter
- Moving average FIR filter (O(1) rolling sum)
- Ladder rung evaluation algorithm
- FBD topological sort (Kahn's algorithm)
- ST recursive descent parser
- ST expression evaluator with short-circuit logic
- Modbus CRC-16 computation (table-driven, O(n))

## L6: Canonical Problems (Complete)
- Conveyor belt start/stop with overload protection
- Traffic light controller (4-phase SFC)
- Water tank level PID control
- Motor starter control circuit
- Pulse timing and edge detection in ladder
- Sequential batch process via SFC

## L7: Applications (Partial — 3 applications)
- Industrial conveyor control (IEC 60204-1 safety)
- Traffic light intersection control
- Water/wastewater tank level control
- Modbus RTU/TCP frame integrity (CRC-16)
- 4-20mA loop diagnostics (NAMUR NE43)
- Process simulation (FOPDT model)

## L8: Advanced Topics (Partial — 2 topics)
- 1oo2 hot standby redundancy (bumpless transfer)
- IEC 61508 SIL assessment (PFD computation)
- 2oo3 TMR architecture
- Safety PLC architecture

## L9: Research Frontiers (Partial — documented)
- OPC UA for PLC data access (referenced)
- Time-Sensitive Networking (TSN) for deterministic PLC comms
- Edge-computing PLCs with ML inference
- AI-assisted PLC programming (natural language to IEC 61131-3)
