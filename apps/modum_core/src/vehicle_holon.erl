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
-module(vehicle_holon).
-behaviour(gen_server).

-export([start_link/3, stop/1]).
-export([init/1, handle_call/3, handle_cast/2,
         handle_info/2, code_change/3, terminate/2]).
-include("states.hrl").
-include_lib("blackboard/include/blackboard_records.hrl").
% sample period to create intention ants.
-define(createIntentionDelay, 20000).
-define(createExplorerDelay, 5000).
-define(solutionEvaporation, 6000).
-define(intentionEvaporation, 300000).
-define(selectIntentionDelay, 15000).
-define(changeIntentionThreshold,0.75).

start_link(VehicleState=#vehicleState{id=Id}, Mode, K) when is_atom(Id) ->
    gen_server:start_link({local,Id}, ?MODULE, [VehicleState,Mode,K], []).

stop(Id) when is_atom(Id) -> 
	gen_server:call(Id, stop, ?callTimeout).

init([VehicleState, Mode, K]) ->
	%% To know when the parent shuts down
    process_flag(trap_exit, true),
	random:seed(erlang:now()),
	BB = bb_ets:create("BB_"++atom_to_list(VehicleState#vehicleState.id)),
	VehicleBeing = #vehicleBeing{state=VehicleState,blackboard=BB,kShortestPaths=?undefined, ksp=K},
	set_mode(Mode),
	{ok, VehicleBeing}.
 
set_mode(?interactive) ->
	timer:send_interval(?createIntentionDelay, createIntention),
 	timer:send_interval(?createExplorerDelay, createExplorer);
set_mode(?auto) ->
	timer:send_interval(?createIntentionDelay, createIntention),
 	timer:send_interval(?createExplorerDelay, createExplorer),
	timer:send_interval(?selectIntentionDelay, selectIntention);
set_mode(?auto_once) ->
	timer:send_after(?createIntentionDelay, createIntention),
 	timer:send_after(?createExplorerDelay, createExplorer),
	timer:send_interval(?selectIntentionDelay, selectIntention).

handle_call(stop, _From, S=#vehicleBeing{}) ->
    {stop, normal, ok, S};
handle_call(_Message, _From, S) ->
    {noreply, S}.

handle_cast(_Message, S) ->
    {noreply, S}.

% callback to handle createIntention message. This is periodically called to create intention ants.
handle_info(createIntention, VB=#vehicleBeing{currentIntention=?undefined}) ->
	{noreply, VB};
handle_info(createIntention, VB=#vehicleBeing{state=#vehicleState{departureTime=DepartureTime,id=VehicleId,location=Location},currentIntention=CurrentIntention}) ->
 	% io:format("Creating intention ant with intention:~w~n", [CurrentIntention]),
	traffic_ant:create_intention_ant(DepartureTime,Location,VehicleId, CurrentIntention),
	{noreply, VB};

handle_info({setCurrentIntention, Pid}, VB) ->
	% fetch solution with id Id
	% io:format("Setting current intention~n"),
	Solution = (pheromone:get_info(Pid))#info.data,
	NewVB = VB#vehicleBeing{currentIntention=Solution},
	{noreply, NewVB};

handle_info({getSolutions, Pid},VB=#vehicleBeing{blackboard=BB}) ->
	S = bb_ets:get_pheromones(BB, [solution]),
	Pid ! {?reply,getSolutions,S},
	{noreply, VB};

handle_info({getTravelTime, SolutionPid, Pid}, VB) ->
	[{First,_}|Solution] = (pheromone:get_info(SolutionPid))#info.data,
	[{Last,_} |_] = lists:reverse(Solution),
	{FirstTime, _} = First#antState.data,
	{_,LastTime} = Last#antState.data,
	Pid ! {?reply,getTravelTime, LastTime-FirstTime},
	{noreply, VB};

handle_info(createExplorer, VB=#vehicleBeing{state=#vehicleState{arrivalTime=?undefined, departureTime=DepartureTime,id=VehicleId,origin=Origin,destination=Destination}, kShortestPaths=?undefined, ksp=KSP}) ->
	{_Lengths,Paths} = lists:unzip(modum_proxy:get_k_shortest_path(Origin#location.resource, Destination#location.resource, KSP)),
	send_update({create_explorers, Paths}, VB),
	lists:foreach(fun(P) -> traffic_ant:create_explorer_ant({ksp,P},DepartureTime, Origin, Destination, VehicleId) end, Paths),
	{noreply, VB#vehicleBeing{kShortestPaths=Paths}};
handle_info(createExplorer, VB=#vehicleBeing{state=#vehicleState{arrivalTime=?undefined,departureTime=DepartureTime,id=VehicleId,origin=Origin,destination=Destination}, kShortestPaths=Paths}) ->
	lists:foreach(fun(P) -> traffic_ant:create_explorer_ant({ksp,P},DepartureTime, Origin, Destination, VehicleId) end, Paths),
	{noreply, VB};
handle_info(createExplorer, VB=#vehicleBeing{state=#vehicleState{arrivalTime=ArrivalTime, departureTime=?undefined,id=VehicleId,origin=Origin,destination=Destination}, kShortestPaths=?undefined, ksp=KSP}) ->
	{_Lengths,Paths} = lists:unzip(modum_proxy:get_k_shortest_path(Origin#location.resource, Destination#location.resource, KSP)),
	lists:foreach(fun(P) -> traffic_ant:create_explorer_ant({upstream,P},ArrivalTime, Destination, Origin, VehicleId) end, Paths),
	{noreply, VB#vehicleBeing{kShortestPaths=Paths}};
handle_info(createExplorer, VB=#vehicleBeing{state=#vehicleState{arrivalTime=ArrivalTime,departureTime=?undefined,id=VehicleId,origin=Origin,destination=Destination}, kShortestPaths=Paths}) ->
	lists:foreach(fun(P) -> traffic_ant:create_explorer_ant({upstream,P},ArrivalTime, Origin, Destination, VehicleId) end, Paths),
	{noreply, VB};
	
% callback to handle newSolution message.
handle_info({newSolution, Solution}, VB=#vehicleBeing{blackboard=BB}) ->
	%io:format("newSolution~n"),
    % TODO: compare new solution to current intention using expit and draw random number
	pheromone:create([BB],?solutionEvaporation, #info{data=Solution,tags=[solution]}),
	{noreply, VB};

handle_info(selectIntention, VB=#vehicleBeing{blackboard=BB,currentIntention=Current}) ->
	
	S = bb_ets:get_pheromones(BB, [solution]),
	Pids = lists:map(fun([A,_])->A end,S),
	ChangeIntention = random:uniform(),
	% change intention?
	L = length(Pids),
	if 
		(L > 0) and ((ChangeIntention > ?changeIntentionThreshold) or (Current =:= ?undefined)) ->
			I = random:uniform(L), % TODO: replace random by optimised based on travel time (stop - start in solution) or logit model on set of solution
			Pid = lists:nth(I, Pids),
			case pheromone:get_info(Pid) of
				?undefined -> {noreply,NewVB} = handle_info(selectIntention, VB);
				PheromoneInfo -> Solution = PheromoneInfo#info.data,
								%io:format("Current intention set to: ~w~n",[Solution]),
								send_update({currentIntention,Solution}, VB),
								NewVB = VB#vehicleBeing{currentIntention=Solution}
			end;
		true ->
			NewVB = VB
	end,
 	{noreply, NewVB};
	
handle_info({subscribe, get_info, Time, once, Receiver}, VB) ->
	timer:send_after(Time, {get_info, Receiver}),
	{noreply, VB};
handle_info({subscribe, get_info, Interval, repeat, Receiver}, VB) ->
	timer:send_interval(Interval, {get_info, Receiver}),
	{noreply, VB};
handle_info({subscribe, get_update, Receiver}, VB=#vehicleBeing{subscribers=Subs}) ->
	case lists:member({get_update, Receiver}, Subs) of
		true -> io:format("Process ~w has already subscribed!~n", [Receiver]), {noreply, VB};
		false ->
			io:format("Process ~w added to subscribers!~n", [Receiver]), 
			NewVB = VB#vehicleBeing{subscribers=[{get_update, Receiver} | Subs]},
			{noreply, NewVB}
	end;
handle_info({unsubscribe, get_update, Receiver}, VB=#vehicleBeing{subscribers=Subs}) ->
	io:format("Unsubscribing process ~w~n", [Receiver]),
	NewSubs = lists:delete({get_update, Receiver}, Subs),
	NewVB = VB#vehicleBeing{subscribers=NewSubs},
	{noreply, NewVB};
handle_info({get_info, Pid},  VB=#vehicleBeing{state=#vehicleState{id=Id}}) ->
	Pid ! {info, Id, VB},
	{noreply, VB};
handle_info(_Message, S) ->
    {noreply, S}.
	
send_update(Info, #vehicleBeing{subscribers=Subs, state=#vehicleState{id=Id}}) ->
	lists:foreach(fun({get_update, Sub}) -> io:format("Sending update to subscriber ~w!~n", [Sub]),Sub ! {update, Id, Info} end, Subs).
	
code_change(_OldVsn, State, _Extra) ->
	{ok, State}.

terminate(normal, S) ->
    io:format("Vehicle ~s terminated normally~n",[(S#vehicleBeing.state)#vehicleState.id]);
terminate(shutdown, _S) ->
    ok;% io:format("Vehicle ~s got shutdown~n",[(S#vehicleBeing.state)#vehicleState.id]);
terminate(Reason, S) ->
    io:format("Vehicle ~s got killed with reason: ~n~w~n", [(S#vehicleBeing.state)#vehicleState.id, Reason]).