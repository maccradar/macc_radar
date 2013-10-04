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
%% Description: TODO: Add description to BlackBoard
-module(bb_ets).


%%
%% Include files
%%
-include("blackboard_records.hrl").
-include_lib("util/include/consts.hrl").
%%
%% Exported Functions
%%
-export([create/1,loop/1,get_pheromones/2]).


get_table_owner([{owner,Pid}|_])->
	Pid;
get_table_owner([_|Items])->
	get_table_owner(Items);
get_table_owner([])->
	undefined. 
create(Name)->
	Table = list_to_atom("blackboard_"++Name),
	case ets:info(Table) of
		undefined -> ets:new(Table, [named_table,bag,{keypos,1}]),
					 Blackboard = spawn_link(bb_ets,loop,[Table]),
					 ets:give_away(Table,  Blackboard, void);
		_ -> Blackboard = get_table_owner(ets:info(Table)),
			 Blackboard!reset
	end,
	Blackboard.

loop(Table)->
	receive
        terminate->void;%%TODO
		
		{insert,Pheromone} -> 
			% TODO: remove assumption Pheromone info is an #info record!!
			Info = pheromone:get_info(Pheromone),
			ets:insert(Table, list_to_tuple([Pheromone|[Info#info.data|Info#info.tags]])),
							 loop(Table);
		{evaporate,{Pheromone,_}}-> ets:delete(Table,Pheromone),
									 loop(Table);
		{getData,Pid,From} when is_pid(Pid) ->
			[P |[]] = ets:lookup(Table, Pid),
			D = element(2,P),
			From ! {?reply,getData,D},
			loop(Table);
		{get,Pattern,From}->Pheromones = ets:match(Table, Pattern),
							From!{result_get,Pheromones},
							loop(Table);
		reset-> ets:delete_all_objects(Table),
				loop(Table);
		_ -> loop(Table)
    end.

get_pheromones(Blackboard,Tags)->
	case is_process_alive(Blackboard) of
		false->?undefined;
		true->Blackboard ! {get,list_to_tuple(['$1'|['$2'|Tags]]),self()},
				receive
					{result_get,Pheromones}->Pheromones
				after 2000->get_pheromones(Blackboard,Tags)
				end
	end.