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
%% Created: 14-jan.-2013
%% Description: TODO: Add description to bb_trafficflow
-module(bb_trafficflow).
%%
%% Include files
%%
-include("blackboard_records.hrl").

-record(blackboard,{flow=[],pheromones}).
%%
%% Exported Functions
%%
-export([create/1,loop/1,request_cumulative/2,init/0]).
-include_lib("util/include/debug.hrl").
%%-include_lib("eqc/include/eqc.hrl").

%% test()->
%%	numtests(1000,?FORALL(X,choose(0,10000),is_number(1/(1000-X)))).

get_table_owner([{owner,Pid}|_])->
	Pid;
get_table_owner([_|Items])->
	get_table_owner(Items);
get_table_owner([])->
	undefined. 
init()->
	?CREATE_DEBUG_TABLE.
create(Name)->
	
	Table = list_to_atom("blackboard_"++Name),
	case ets:info(Table) of
		undefined -> ets:new(Table, [named_table,bag,{keypos,2}]),
					 Blackboard = spawn_link(?MODULE,loop,[#blackboard{pheromones = Table}]),
					 ets:give_away(Table,  Blackboard, void),
					 Blackboard;
		TableInfo -> Blackboard = get_table_owner(ets:info(Table)),
					 Blackboard!reset,
					 Blackboard
	end.


-record(ets_item,{vehicle,pheromone,time}).

loop(BB = #blackboard{flow=Flow,pheromones = Pheromones})->
	receive
		{get_cumulative,From,Ref}->Cumulative = time_buckets_lists:get_cumulative(Flow),
							   From!{updateBlackboard,Ref,Cumulative},
							   loop(BB);
	    terminate->ets:delete(Pheromones);
		{insert,Pheromone} -> 	Info = pheromone:get_info(Pheromone),
								case ets:lookup(Pheromones, Info#vehicle_info.vehicle) of
									[OldPheromone] 	-> 	OldPheromone#ets_item.pheromone ! evaporate,
														ets:delete_object(Pheromones,OldPheromone),
														ets:insert(Pheromones, #ets_item{vehicle= Info#vehicle_info.vehicle,pheromone = Pheromone,time=Info#vehicle_info.arrival_time}),
														NewFlow = time_buckets_lists:replace_item(Flow, OldPheromone#ets_item.time,Info#vehicle_info.arrival_time, OldPheromone#ets_item.pheromone,Pheromone, void),
														loop(BB#blackboard{flow=NewFlow,pheromones = Pheromones});														
									OldPheromones 	-> 	EVAP = fun (OldPheromone,FlowAcc) -> 	OldPheromone#ets_item.pheromone ! evaporate,
																								ets:delete_object(Pheromones,OldPheromone),
																								time_buckets_lists:remove_item(FlowAcc, OldPheromone#ets_item.time, OldPheromone#ets_item.pheromone)
														end,
														NewFlow = lists:foldl(EVAP, Flow,OldPheromones),
														NewFlow2 = time_buckets_lists:add_item(NewFlow,Info#vehicle_info.arrival_time, Pheromone, void),
														ets:insert(Pheromones, #ets_item{vehicle= Info#vehicle_info.vehicle,pheromone = Pheromone,time=Info#vehicle_info.arrival_time}),
														loop(BB#blackboard{flow=NewFlow2,pheromones = Pheromones})
								end;								
		{evaporate,{Pheromone,Info}}-> case ets:match_object(Pheromones, {'_',Info#vehicle_info.arrival_time,Pheromone}) of
										   [] -> loop(BB#blackboard{flow=Flow,pheromones = Pheromones});
										   _ -> NewFlow = time_buckets_lists:remove_item(Flow, Info#vehicle_info.arrival_time, Pheromone),
												ets:delete_object(Pheromones,#ets_item{vehicle= Info#vehicle_info.vehicle,pheromone = Pheromone,time=Info#vehicle_info.arrival_time}),
												loop(BB#blackboard{flow=NewFlow,pheromones = Pheromones})
									   end;
	
		reset-> ets:delete_all_objects(Pheromones),
				loop(#blackboard{flow=Flow,pheromones=Pheromones});
		_Message ->loop(BB)

    end.

request_cumulative(Blackboard,Ref)->
	Blackboard ! {get_cumulative,self(),Ref}.