% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
%                                                                                                         %
%  This file is created as part of research done by the Multi-Agent Coordination & Control (MACC) group.  %
%                                                                                                         %
%  (C) 2013 Johan Philips, Bart Saint Germain, Jan Van Belle, Paul Valckenaers, macc.radar@gmail.com      %
%                                                                                                         %
%  Department of Mechanical Engineering, Katholieke Universiteit Leuven, Belgium.                         %
%                                                                                                         %
%  You may redistribute this software and/or modify it under either the terms of the GNU Lesser           %
%  General Public License version 2.1 (LGPLv2.1 <http://www.gnu.org/licenses/old-licenses/lgpl-2.1.html>) %
%  or (at your discretion) of the Modified BSD License:                                                   %
%  Redistribution and use in source and binary forms, with or without modification, are permitted         %
%  provided that the following conditions are met:                                                        %
%     1. Redistributions of source code must retain the above copyright notice, this list of              %
%        conditions and the following disclaimer.                                                         %
%     2. Redistributions in binary form must reproduce the above copyright notice, this list of           %
%        conditions and the following disclaimer in the documentation and/or other materials              %
%        provided with the distribution.                                                                  %
%     3. The name of the author may not be used to endorse or promote products derived from               %
%        this software without specific prior written permission.                                         %
%  THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING,    %
%  BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE     %
%  ARE DISCLAIMED. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,  %
%  EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS    %
%  OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY        %
%  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR           %
%  OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY      %
%  OF SUCH DAMAGE.                                                                                        %
%                                                                                                         %
% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
-include_lib("util/include/consts.hrl").

-define(interactive,interactive).
-define(manual, manual).
-define(auto, auto).
-define(auto_once, auto_once).
-define(user_request_server, user_request_server).
-define(user_response_client, user_response_client).
-define(web_server, web_server).
-define(forecast_server, forecast_server).
-define(optimal_path_server, optimal_path_server).
-define(traffic_update_server, traffic_update_server).
-define(traffic_update_client, traffic_update_client).

-record(comState,
{
	init,
	ip,
	port,
	parser
}).

% defines a connection on a node holon from one link holon to another link holon
% or a connection on a link holon from one node holon to another node holon.
% from and to links have to be specified by either their pid or a registered name (atom)
% since these are used to send messages to the relevant link holons.
-record(connection,
{
	from										::atom() | pid(),
	to											::atom() | pid()
}).

% defines a set of models used by the resource holons (nodes & links)
% the fd model represents the fundamental diagram of traffic flow (traffic flux (vehicles/s) vs. traffic density (vehicles/m))
% the ttm model represents the travel time model to be used in the particular resource.
% the stm model represents the start time model to be used to determine the latest start time given an end time.
-record(models,
{
	fd											::any(),
	ttm											::any(),
	stm											::any()
}).

% defines a blackboard for storing traffic data at the begin (bb_b) and end (bb_e) of a link holon. Also cumulative functions
% at begin (cf_b) and end (cf_e) are stored in this record. 
-record(blackboard,
{
	bb_b										::any(),
	bb_e										::any(),
	cf_b										::any(),
	cf_e										::any()
}).

-record(location,
{
	resource			=?undefined				::atom() | pid(),
	position			=?undefined				::atom() | pid()
}).

% defines the internal state of a vehicle holon. This state contains the registered name (atom) or pid as unique identifier (id), its origin
% and destination, both indicated by the registered name or pid of that resource holon, the current resource it resides on and optional vehicle 
% information such as length and maximum speed.
-record(vehicleState,
{
	id											::atom() | pid(),
	origin				=#location{}			::#location{},
	destination			=#location{}			::#location{},
	location			=#location{}			::#location{},
	length				=1.0					::float(),
	maxSpeed			=120.0					::float(),
	departureTime		=now					::atom() | float(),
	arrivalTime			=?undefined				::atom() | float()
}).

% defines the vehicle being, containing, for now, only the internal state of the vehicle holon.
-record(vehicleBeing,
{
 	state				=#vehicleState{}		::#vehicleState{},
	blackboard									::any(),
	currentIntention	=?undefined				::any(),
	kShortestPaths		=?undefined             ::list() | ?undefined,
	ksp					= 3						::integer(),
	subscribers			=[]						::list(pid())
}). 

-record(point,
{
	x					=0.0					::float(),
	y					=0.0					::float()
}).

-record(nodeState,
{
	id											::atom() | pid(),
	desc										::list(char()),
	connections			=[#connection{}]		::list(#connection{}),
	shape				=[#point{}]				::list(#point{}),
	capacities									::dict(),
	coordinates			=[]						::list({float(),float()}), % [{lat, lon}]
	turningFractions							::dict() % key: {from,to}, value: turning fraction 
}). 

% defines the node being, containing the internal state and set of models used by the node holon.
-record(nodeBeing, % if nodeBeing requires a blackboard, convert nodeBeing and linkBeing to Being...
{
	state				=#nodeState{}			::#nodeState{},
	models				=#models{}				::#models{}
}).

% defines the internal state of a link holon. This state contains the registered name (atom) or pid as unique identifier (id),
% the connection defining which two nodes this link connects, and other static information such as its number of lanes (numLanes),
% length and maximum allowed speed. Also dynamic link information, such as co2 emissions, traffic density and average speed, are 
% stored in this state record. 
-record(linkState,
{
	id											::atom(),
	desc										::list(char()),
	connection			=#connection{}			::#connection{},
	numLanes			=0						::integer(),
	length				=0.0					::float(),					% [m]
	shape				=[#point{}]				::list(#point{}),
	maxAllowedSpeed		=0.0					::float(),					% [m/s]
	co2emissions		=0.0					::float(),
	density				=0.0					::float(),					% [#cars/m]
	avgSpeed			=0.0					::float(),					% [m/s]
	roadType									::string(),
	coordinates									::list({float(),float()})   % [{lat, lon}]
}).

% defines the link being, containing the internal state, set of models and blackboard used by the link holon.
-record(linkBeing,
{
	state				=#linkState{}			::#linkState{},
	blackboard			=#blackboard{}			::#blackboard{},
	models				=#models{}				::#models{}
}).

% defines the internal state of the modum proxy module, responsible for the communication between the UTMC server
% and the traffic radar. It contains 2 dictionaries with dynamic information on node and link holons, received from
% the UTMC server, a model defining the communication protocol (xml) and a graph representing the topology of nodes
% and links.
-record(proxyState,
{
	nodeInfoDict								::dict(),
	linkInfoDict								::dict(),
	model										::any(),
    graph										::digraph(),
	nodes										::list(atom()),
	links										::list(atom()),
	clients										::list({atom(),pid()}),
	servers		 								::list({atom(),pid()}),
	cache										::dict() % Key: {atom(),atom()}, Value: [list(atom())] cache for ksp, list of link ids of a particular type, ...
}).

% defines the internal state of an ant agent, containing its current resource it is operating on, the id of the vehicle, creating this ant, and the corresponding time slot.
% the current resource should be specified by a registered name or pid, since it is used to send messages to the resource holon.
-record(antState,
{
 	location			=#location{}			::#location{},
	vehicleId									::atom() | pid(),
	time				=?undefined				::float() | ?undefined
}).

% defines a generic scenario, containing a time, ant state and boundary condition
-record(scenario,
{
	timeSlot			={?undefined,?undefined}::{float() | ?undefined , float() | ?undefined },
    antState									::#antState{},
	boundaryCondition	= end_link				::any() %TODO {Resource, Pos}
}).

% defines a generic ant agent, containing a function to create scenarios and a function to execute scenarios,
% the internal ant state and a list of ant states, representing the history of the corresponding ant.
-record(ant,
{
	executeScenario									::fun((#scenario{}) -> #antState{}),
	createScenario									::fun((#antState{}, list(#antState{})) -> #scenario{}),
	state											::#antState{},
	history											::list({#antState{},#scenario{}}),
	hopLimit										::integer()
}).