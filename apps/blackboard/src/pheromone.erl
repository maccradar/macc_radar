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
%% Author: bgermain
%% Created: 17-dec.-2012
%% Description: TODO: Add description to pheromone
-module(pheromone).


-export([create/3,loop/4,get_info/1]).

-include_lib("util/include/debug.hrl").


%% timeout in msec...
create(Blackboards,TimeOut,Info)->
	Pheromone = spawn(?MODULE, loop, [Blackboards,erlang:now(),TimeOut,Info]),
	register_to_blackboards(Pheromone,Blackboards),
	
	Pheromone.

register_to_blackboards(_,[])->void;
register_to_blackboards(Pheromone,[Blackboard|Tail])->
	Blackboard!{insert,Pheromone},
	register_to_blackboards(Pheromone,Tail).
evaporate(_,_,[])->void;
evaporate(Pheromone,Info,[Blackboard|Tail])->
	Blackboard!{evaporate,{self(),Info}},
	evaporate(Pheromone, Info, Tail).


timediff( {A1, B1, C1},{A2, B2, C2}) ->
    ((A2-A1)*1000000 + B2-B1)*1000000 + C2-C1.

time_to_evaporation(CreationTime,TimeOut)->
	Diff = timediff(CreationTime,erlang:now()) div 1000,
	max(TimeOut - Diff,0).

loop(Blackboards,CreationTime,TimeOut,Info)->
	Time_to_Evaporation = max(0,time_to_evaporation(CreationTime,TimeOut)),
    receive
		{get_info,From}->
			From!{info,Info},
			loop(Blackboards,CreationTime,TimeOut,Info);
		evaporate->evaporate(self(), Info, Blackboards);
		Message -> io:format("unknown message to pheromone ~w ~n",[Message])
    after Time_to_Evaporation->evaporate(self(), Info, Blackboards)					   
    end.
	
get_info(Pid)->
	case is_process_alive(Pid) of
		false->undefined;
		true->Pid ! {get_info,self()},
				receive
					{info,Info}->Info
				after 2000->get_info(Pid)
				end
	end.