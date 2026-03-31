BFWS with Intention Based Heuristics
=====
Code for BFWS planners that implement the intention-based heuristics proposed in [**Goal Recognition as Planning: Deriving Heuristics from Intention Models**](https://arxiv.org/abs/2603.14824), to be presented at ICAPS 2026. 
This code extends [LAPKT-Public](https://github.com/LAPKT-dev/LAPKT-public), and [LAPKT-BFNoS](https://github.com/grosa97/LAPKT-BFNoS).


<!-- <ins>**Installation** -->
Installation
-

Use Ubuntu >= 20, recommend later versions (22, 24) as cmake >= 3.19 is required.

Build LAPKT by following the [Build instructions](developersguide/build.md). It is **required to also install the Tarski parser and grounder for this solver**, as described in the developers guide. Even though the FD grounder is used by default, LAPKT currently does not support axioms produced by the FD grounder for a small number of problems, and in such cases it automatically detects the axioms and falls back to the Tarski grounder.

**Important**: Additional requirements before building.

- Update submodule, run from project directory:
		
		git submodule update --init --recursive
	
	This is because *external_package/fd_translate* was updated compared to the main LAPKT 2.0 repo.
	
- Also required for build:
	
		sudo apt install python-is-python3
	
	Because somewhere in the new _fd_translate_ files, _python_ is used instead of _python3_, so this is a suboptimal and temporary fix.


Usage
-

**Run $BFWS(f5)-OL_r$ solver with FD grounder after building:**

	python3 lapkt_package/lapkt.py BFWS --grounder FD -d <domain> -p <problem> --search_type BFWS-OL [options]

   	Options:
      --time_limit [int]		Memory threshold in MB, 0 is no threshold (default: 0).
      --memory_limit [int]		Time threshold in seconds, 0 is no threshold (default: 0).
      --rpop_num_samples [int]	Number of sample relaxed plans extracted when using OL or OP (default: 100).
      --rpop_max_ms [int]		Max time sampling relaxed plans for OL or OP in ms (default: 10000).
      --seed [int]			Trimmed open list and OL/OP seed (default: 42).
      --tol_max_depth [int]		Trimmed open list max depth (default: 18).
      --normal_ol			Flag. Uses normal open list instead of trimmed open list.
      --goalcount 			Flag. Substitutes hlm for goalcount heuristic for following variants.
      

**Run $BFWS(f5)-OL_r-UTP$ solver with FD grounder after building:**

	python3 lapkt_package/lapkt.py BFWS --grounder FD -d <domain> -p <problem> --search_type BFWS-OL-UTP [options]

**Run $BFWS(f5)-OL_r-DUAL$ solver with FD grounder after building:**

	python3 lapkt_package/lapkt.py BFWS --grounder FD -d <domain> -p <problem> --search_type BFWS-OL-DUAL [options]
 
Note: time_limit and memory_limit refer to the frontend solver thresholds when using $BFWS(f5)-OL_r-DUAL$.
 
**Run $BFWS(f5)-OL_r$ solver with normal open list and goal counting, as Table 1:**

	python3 lapkt_package/lapkt.py BFWS --grounder FD -d <domain> -p <problem> --search_type BFWS-OL --normal_ol --goalcount[options]

**Run $BFWS(f5)-OP_r$ solver with normal open list and goal counting, as Table 1:**

	python3 lapkt_package/lapkt.py BFWS --grounder FD -d <domain> -p <problem> --search_type BFWS-OP --normal_ol --goalcount[options]

Implementation details
-
<!-- <ins>**Implementation details** -->

- OL heuristic is implemented in the `src/node_eval/heuristics/rp_graph.hxx` file.
- OL-UTP variant is in `src/node_eval/heuristics/rp_graph_utp.hxx` file.
- The OP heuristic is in `src/node_eval/heuristics/rp_graph_op.hxx` file.
- OL variants use the planner engine `src/engine/gs_bfws_3h.hxx`.
- OP variant uses the planner engine `src/engine/gs_bfws_3h_op.hxx`. 

- The information extraction phase is computed at the beginning of the `start()` function in the engine files.
- The online probability heuristics are computed in the `rp_fl_achieved(...)` function in the engine files.



Authors
-

- Giacomo Rosa <giacomo.rosa@student.unimelb.edu.au>
- Nir Lipovetzky <nir.lipovetzky@unimelb.edu.au>
- Jean Honorio <jean.honorio@unimelb.edu.au>
- Sebastian Sardina <sebastian.sardina@rmit.edu.au>



LAPKT
=====


![LAPKT](cmake/docs/resources/logo/lapkt-low-resolution-logo-color-on-transparent-background.png)

LAPKT aims to make your life easier if your purpose is to create, use or extend basic to advanced Automated Planners. It's an open-source Toolkit written in C++ and Python with simple interfaces that give you complete flexibility by decoupling parsers from problem representations and algorithms. It has been succesfully used in embedded systems, webservices, compilations, replanning and contains some of the high-performance planners from the last International Planning Competition 2014.

AUTHORS
=======

- Miquel Ramirez <miquel.ramirez@gmail.com>
- Nir Lipovetzky <nirlipo@gmail.com>
- Anubhav Singh <anubhav.singh.er@protonmail.com>
- Christian Muise <christian.muise@gmail.com>



OVERVIEW
===========

LAPKT separates search engines from the data structures used to represent
planning tasks. This second component receives the name of 'interface' since
it is indeed the interface that provides the search model to be solved.

Search engine components are meant to be modular, allowing users of LAPKT to
assemble and combine features of different search engines to come up with customized
search strategies, within reason and without sacrificing (much) efficiency. In order to
do so, LAPKT makes heavy use of C++ templates and the Static Strategy design pattern.
At the time of writing this, the modularity and decoupling of components isn't as high 
as I would like, there's still a lot of work to do :)

LAPKT is bound to change rapidly and dramatically over the next months, so please keep
this in mind when using this library.


## Important platform requirements:

**Platform agnostic**

1. The directory where the `pip` command installs the scripts, including `lapkt_cmd.py`, is generally on the system `PATH`, if not, it needs to be added manually.
2. Python version [ `3.7`, `3.7`, `3.8`, `3.9`, `3.10` ] are supported

**@Windows**

1. `clingo/gringo` python package requires `MSVCP140.dll` which comes with visual studio redistributable. [latest vc-redist](https://docs.microsoft.com/en-us/cpp/windows/latest-supported-vc-redist)
2. To be able to run `lapkt_run.py` script directly from command line, change the default handler for ".py" files to `Python'.


Introduction to LAPKT 4 Devs
================================

[Build instructions](developersguide/build.md)

`cmake` is the primary tool used to build the LAPKT's C++(backend) source code. We also use it to generate Python/C++ library package which is ready to go as a `pypi` package. 

