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
%% Created: 4-jan.-2013
%% Description: TODO: Add description to test
-module(bb_test).

-export([pheromone/1,pheromone_sub/2,create_pheromones/4]).
-include("blackboard_records.hrl").
-include_lib("util/include/debug.hrl").

 generate_info(bb_ets,ID,_)->
 	#info{data=void,tags= [een,twee,ID]};
generate_info(bb_trafficflow,ID,{Arrival_From,Arrival_To})->
	#vehicle_info{vehicle={Arrival_From,ID},arrival_time=Arrival_From + random:uniform(Arrival_To - Arrival_From),data=void}.
	

create_pheromones(_,_,0,_)->
	void;
create_pheromones(BB_Module,BB,Nbr,Data)->
	pheromone:create([BB], 60000, generate_info(BB_Module, Nbr,Data)),
	create_pheromones(BB_Module,BB, Nbr-1,Data).

%%BB = test:pheromone(bb_trafficflow).
%%bb_trafficflow:get_cumulative(BB).
%%list_to_pid("een PID_name")!evaporate.
%%tv:start(). 

pheromone(BB_Module) when is_atom(BB_Module) ->
	BB = BB_Module:create("bb"),
	create_pheromones(BB_Module,BB,1000,{0,10}),
	create_pheromones(BB_Module,BB,500,{10,3600}),
	create_pheromones(BB_Module,BB,3000,{3600,7200}),

	BB;

pheromone(BB) ->
	
	BB_Module = bb_trafficflow,
	create_pheromones(BB_Module,BB,1000,{0,10}),
	create_pheromones(BB_Module,BB,500,{10,3600}),
	create_pheromones(BB_Module,BB,3000,{3600,7200}),
	BB.
pheromone_sub(BB,0)->
	bb_ets:get_pheromones(BB, {een,'_'});
pheromone_sub(BB,Nbr)->
	bb_ets:get_pheromones(BB, {een,'_'}),
	pheromone_sub(BB, Nbr-1).
	
