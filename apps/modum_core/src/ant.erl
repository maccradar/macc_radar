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
% a generic ant agent. its behaviour is determined by the createScenario and executeScenario functions
-module(ant).

% public
-export([create/1,create/2]).
% private
-export([loop/1,start/2]).
-include("states.hrl").
% create an ant agent by specifying an ant record containing:
% - createScenario and executeScenario functions specifying its behaviour on the resources
% - history, e.g. of resources the ant has already been too, typically an empty list at creation
% - state containing its current resource and time slot
create(Ant=#ant{}) ->
	spawn(ant,loop,[Ant]).

create(Ant=#ant{},Init) when is_function(Init) ->
	spawn(ant,start,[Ant,Init]);
create(Ant=#ant{},_) ->
	spawn(ant,loop,[Ant]).

start(Ant,Init)	->
	Init(),
	loop(Ant).

% internal loop of an ant agent. basically this creates a scenario (using the createScenario function),
% then executes this scenario (using the executeScenario function) and then repeats these steps until
% either no new scenario can be created and the ant reaches its end of life or either no meaningful state
% could be retrieved after executing the scenario, also resulting in the ant's death.
loop(Ant=#ant{createScenario=CS, executeScenario=ES,history=H,state=S,hopLimit=HL}) ->
	Scenario = CS(S,H),
	NewState = ES(Scenario),
	%(HL >= length(H)) orelse io:format("History: ~w~n",[[(AS#antState.location)#location.resource || {AS,_} <- H]]),
	((NewState == ?undefined) or (HL < length(H))) orelse
	loop(Ant#ant{history=[{S,Scenario}|H],state=NewState}).