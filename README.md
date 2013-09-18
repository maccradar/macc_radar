MACC RADAR
==========

The MACC RADAR project provides a multi agent system for short-term forecasts developed in Erlang/OTP
by the Multi Agent Coordination and Control group of the Department of Mechanical Engineering at the 
University of Leuven (KU Leuven).

The goal is to provide a generic forecasting tool which can be used in various application domains.
The implementation at the moment, however, is focussed on intelligent transportation.

For more detailed information on the scientific research behind this project, following references are
recommended:

- Architecture on which this work is based:
"Reference Architecture for Holonic Manufacturing Systems: PROSA",
Hendrik Van Brussel, Jo Wyns, Paul Valckenaers, Luc Bongaerts, Patrick Peeters,
Computers In Industry, special issue on intelligent manufacturing systems,
Vol. 37, No. 3, pp. 255 - 276, 1998.

- Overview of application domains:
"A Service-Oriented Approach for Holonic Manufacturing Control and Beyond",
Jan Van Belle, Johan Philips, Osman Ali, Bart Saint Germain, Hendrik Van Brussel, Paul Valckenaers,
Service Orientation in Holonic and Multi-Agent Manufacturing Control,
Chapt. 1 Springer, 1-20, 2012

- MACC RADAR applied in intelligent transportation:
"Traffic Radar: A Holonic Traffic Coordination System Using PROSA++ and D-MAS",
Johan Philips, Bart Saint Germain, Jan Van Belle, Paul Valckenaers,
Industrial Applications of Holonic and Multi-Agent Systems,
Berlin Heidelberg: Springer-Verlag, 163-174, 2013.

Installation instructions (Linux)
---------------------------------
0. Get and install Erlang/OTP  R14 or higher (https://www.erlang-solutions.com/downloads/download-erlang-otp)
1. Get and install Rebar:
# apt-get install rebar
2. Clone MACC RADAR using git:
$ git clone https://github.com/maccradar/macc_radar.git
2. Get dependencies using Rebar:
$ cd macc_radar
$ rebar get-deps
3. Open macc_rader/apps/modum_core/src/modum_core.app.src with your favorite editor and change environment variable
"project_dir" into the location of your macc_radar installation. E.g. /home/user/projects/macc_radar/
$ nano macc_radar/apps/modum_core/src/modum_core.app.src
change this line: {env, [{project_dir, "/research/workspace/Erlang/macc_radar"},
4. Compile using Rebar:
$ cd macc_radar
$ rebar compile
5. Run the program using the shell script:
$ sh ./modum_run.sh
6. Open a browser and check if the web service is running, default this would be at http://localhost:9000/ but you can
edit these settings in the modum_core.app.src file
7. Run traffic simulations :-)
