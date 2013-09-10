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
% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
% API to create traffic ants (explorer, intention, flow) with particular behaviours (random, shortest path, heuristic) %
% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
-module(traffic_ant).

% public
-export([create_explorer_ant/5, create_intention_ant/4]).
-include("states.hrl").
-include_lib("util/include/debug.hrl").

create_explorer_ant(Type = random,CurrentTime, CurrentLocation, Destination, VehicleId) ->
	AntState = #antState{location=CurrentLocation,vehicleId=VehicleId, time=CurrentTime},
	CreateScenario = 
		(fun(Cur_AntState=#antState{location=Current,time=Time}, History) ->
			case Current#location.resource == Destination#location.resource of % check if destination has been reached.
				true ->
					io:format("Destination reached~n"),
					VehicleId ! {newSolution, lists:reverse([{Cur_AntState,?undefined}|History])}, ?undefined; % return the result
				false ->
					Current#location.resource ! {downstream_connections,self()},
					Connections = receive 
						{?reply,downstream_connections,Downstream_Connections} -> Downstream_Connections;
						_ -> []
					end,
					Nth = random:uniform((length(Connections))),
					NextResource = lists:nth(Nth, Connections),
					#scenario{antState=Cur_AntState,timeSlot={Time,?undefined},boundaryCondition=NextResource}
			end
		end), 
	ExecuteScenario = execute_scenario(explorer, Type),
	ant:create(#ant{createScenario=CreateScenario,executeScenario=ExecuteScenario,state=AntState,history=[],hopLimit=10000},fun()-> random:seed(erlang:now()) end);

create_explorer_ant(Type=heuristic, CurrentTime, CurrentLocation, Destination=#location{resource=DestResource}, VehicleId) ->
	AntState = #antState{location=CurrentLocation,vehicleId=VehicleId, time=CurrentTime},
	DestResource ! {downstream_point,self()},
	DestPoint = 
		receive
			{?reply,downstream_point,Point} -> Point;
			_ -> ?undefined
		end,
	io:format("Destination ~w has point ~w~n",[Destination,DestPoint]),
	CreateScenario = 
		(fun(Cur_AntState=#antState{location=Current,time=Time}, History) -> 
			%io:format("Current: ~w, destination: ~w~n",[Current,Destination]),
			CurResource = Current#location.resource,
			case CurResource of % check if destination has been reached.
				DestResource ->
					io:format("Destination reached!~n"),
					VehicleId ! {newSolution, lists:reverse([{Cur_AntState,?undefined}|History])}, ?undefined; % return the result
				#location{resource=?undefined} ->
					io:format("Current location undefined!~n"),
					?undefined;
				_ ->
					CurResource ! {downstream_point,self()},
					CurPoint = 
						receive
							{?reply,downstream_point,Point_} -> Point_;
							_ -> ?undefined
						end,
					From = case History of
						[{#antState{location=L},_}] -> L;
						[{#antState{location=L},_}|_] -> L;
						_ -> ?undefined
						end,
					%io:format("Requesting downstream connections on ~w from ~w~n",[Current, From]),
					case From of
						?undefined -> CurResource ! {downstream_connections, self()};
						_ -> CurResource ! {downstream_connections, From#location.resource, self()}
					end,
					Connections = receive 
						{?reply,downstream_connections,Downstream_Connections} -> Downstream_Connections;
						_ -> []
					end,
					%io:format("Connections: ~w~n", [Connections]),
					F = fun(C) -> C ! {upstream_point,self()}, receive {?reply, upstream_point, P} -> {C,P} end end,
					% filter out points at the borders
					ConPoints = [{X,Y} || {X,Y} <- lists:map(F,Connections), Y < #point{x=?inf,y=?inf}], % add the filter to heuristic function iso create_ant
					%io:format("Connection points: ~w~n", [ConPoints]),
					NextResource = heuristic(randomized_distance_search,{CurResource, CurPoint},{DestResource,DestPoint},ConPoints),
					%io:format("Next resource:~w~n",[NextResource]),
					#scenario{antState=Cur_AntState,timeSlot={Time,?undefined},boundaryCondition=NextResource}
			end
		end),
	ExecuteScenario = execute_scenario(explorer, Type),
	ant:create(#ant{createScenario=CreateScenario,executeScenario=ExecuteScenario,state=AntState,history=[],hopLimit=100},fun()-> random:seed(now()) end);
create_explorer_ant(Type={ksp,Path},CurrentTime, CurrentLocation, Destination, VehicleId) ->
	%io:format("Creating explorer ant with path ~w~n", [Path]),
	AntState = #antState{location=CurrentLocation,vehicleId=VehicleId, time=CurrentTime},
	CreateScenario = (fun(Cur_AntState=#antState{location=Current,time=Time}, History) -> 
							  case Current#location.resource == Destination#location.resource of % check if destination has been reached.
								  true ->
										%io:format("Explorer reached destination!~n"),
									  VehicleId ! {newSolution, lists:reverse([{Cur_AntState,?undefined}|History])}, ?undefined; % return the result
								  false ->
									  NextResource = lists:nth(length(History)+1, Path),
									  #scenario{antState=Cur_AntState,timeSlot={Time,?undefined},boundaryCondition=NextResource}
							  end
					  end), 
	ExecuteScenario = execute_scenario(explorer,Type),
	ant:create(#ant{createScenario=CreateScenario,executeScenario=ExecuteScenario,state=AntState,history=[]},?undefined);
create_explorer_ant(Type=shortest_path,CurrentTime, CurrentLocation, Destination, VehicleId) ->
	AntState = #antState{location=CurrentLocation,vehicleId=VehicleId, time=CurrentTime},
	Path = create_shortest_path(CurrentLocation,Destination),
	CreateScenario = (fun(Cur_AntState=#antState{location=Current,time=Time}, History) -> 
							  case Current#location.resource == Destination#location.resource of % check if destination has been reached.
								  true ->
									  VehicleId ! {newSolution, lists:reverse([{Cur_AntState,?undefined}|History])}, ?undefined; % return the result
								  false ->
									  NextResource = dict:fetch(length(History), Path),
									  #scenario{antState=Cur_AntState,timeSlot={Time,?undefined},boundaryCondition=NextResource}
							  end
					  end), 
	ExecuteScenario = execute_scenario(explorer,Type),
	ant:create(#ant{createScenario=CreateScenario,executeScenario=ExecuteScenario,state=AntState,history=[]},?undefined);
create_explorer_ant(Type={upstream,Path},EndTime, CurrentLocation, Destination, VehicleId) ->
	[_ | P] = lists:reverse(Path),
	{PathDict, PathSize} = lists:foldl(fun(R,{D,K}) -> {dict:store(K, R, D), K+1} end, {dict:new(),0}, P),
	AntState = #antState{location=CurrentLocation,vehicleId=VehicleId, time=EndTime},
	CreateScenario = (fun(Cur_AntState=#antState{location=Current,time=Time}, History) -> 
							   case Current#location.resource == Destination#location.resource of % check if destination has been reached.
								  true ->
									  io:format("Upstream Explorer reached destination!~n"),
									  VehicleId ! {newSolution, history_to_solution([{Cur_AntState,?undefined}|History])}, ?undefined; % return the result
								  false ->
										dict:is_key(length(History), PathDict) orelse io:format("history too big: ~w, ~w~n",[Current, Destination]),
										PrevResource = dict:fetch(length(History), PathDict),
									    #scenario{antState=Cur_AntState,timeSlot={?undefined,Time},boundaryCondition=PrevResource}
							  end
					  end), 
	ExecuteScenario = execute_scenario(upstream_explorer,Type),
	ant:create(#ant{createScenario=CreateScenario,executeScenario=ExecuteScenario,state=AntState,history=[]},?undefined).	

history_to_solution(History)->
	FixScenario_StartTime = fun ({State = #antState{time = T},Scenario = #scenario{timeSlot = {_,E}}},ST) ->
									{{State#antState{time = ST},Scenario#scenario{timeSlot = {ST,E}}},T};
                                ({#antState{time = T},_},_) ->
									{{?undefined,?undefined},T} end,
    {[_|Solution0],_} = lists:mapfoldl(FixScenario_StartTime,?undefined,History),
    FixScenario_NextResource = fun({State = #antState{location = #location{resource = R}},Scenario},NR) -> {{State,Scenario#scenario{boundaryCondition = NR}},R} end,
    {Solution,_} = lists:mapfoldr(FixScenario_NextResource,?undefined,Solution0),
    Solution.
                      
% TODO: klopt dit nog? intention ant starts at the end of the CurrentResource link at CurrentTime. CurrentResource should then not be part of the solution.
create_intention_ant(CurrentTime, CurrentLocation, VehicleId, Solution) ->
	{SolutionDict, SolutionSize} = lists:foldl(fun(S,{D,K}) -> {dict:store(K, S, D), K+1} end, {dict:new(),0}, Solution),
	AntState = #antState{location=CurrentLocation,vehicleId=VehicleId, time=CurrentTime},
	
	CreateScenario = (fun(Cur_AntState = #antState{location=CurLocation,time=Time},History) -> 
							%io:format("CurrentAntState: ~w, history: ~w~n",[Cur_AntState, History]),
								case dict:fetch(length(History),SolutionDict) of
											{#antState{location=CurLocation},#scenario{boundaryCondition=NextResource}} -> #scenario{antState=Cur_AntState,timeSlot={Time,?undefined},boundaryCondition=NextResource};
											{#antState{location=CurLocation},?undefined} -> 
											%io:format("Intention ant finished~n"),
											?undefined;
											Other -> io:format("error in dict ~w ~n ",[Other]),?undefined
								end
					  end), 
	ExecuteScenario = execute_scenario(intention, normal),
	ant:create(#ant{createScenario=CreateScenario,executeScenario=ExecuteScenario,state=AntState,history=[]}).


execute_scenario(explorer, _Type) ->
	fun(Scenario=#scenario{boundaryCondition= NextResource,antState=#antState{location=#location{resource=Rid},vehicleId=Vid}}) ->
		Rid ! {execute, Scenario, self()},
		receive
			{?reply, execute, EndTime} -> #antState{location=#location{resource=NextResource,position=Rid},vehicleId=Vid,time=EndTime}
		after 5000 -> io:format("No response from execute request for scenario ~w on resource ~w~n",[Scenario, Rid]), ?undefined
		end;
		(?undefined) -> ?undefined
	end;
execute_scenario(upstream_explorer, _Type) ->
	fun(Scenario=#scenario{boundaryCondition= PrevResource,antState=#antState{location=#location{resource=Rid},vehicleId=Vid}}) ->
	% fun(Scenario=#scenario{boundaryCondition= NextResource,antState=#antState{location=#location{resource=Rid},vehicleId=Vid}}) ->
		Rid ! {explore_upstream, Scenario, self()},
		receive
			{?reply, explore_upstream, StartTime} -> #antState{location=#location{resource=PrevResource,position=Rid},vehicleId=Vid,time=StartTime}
		after 5000 -> io:format("No response from execute request for scenario ~w on resource ~w~n",[Scenario, Rid]), ?undefined
		end;
		(?undefined) -> ?undefined
	end;
		
execute_scenario(intention, _Type) ->
	fun(Scenario=#scenario{boundaryCondition= NextResource,antState=#antState{location=#location{resource=Rid},vehicleId=Vid}}) ->
		Rid ! {proclaim, Scenario, self()},
		receive
			{?reply, proclaim, EndTime} -> #antState{location=#location{resource=NextResource,position=Rid},vehicleId=Vid,time=EndTime}
		after 5000 -> io:format("No response from proclaim request for scenario ~w on resource ~w~n",[Scenario, Rid]), ?undefined
		end;
		(?undefined) -> ?undefined
	end;
execute_scenario(_,_) ->
	?undefined.

heuristic(_,_,_,[]) ->
	?undefined;
heuristic(_,_,_,[{NextId,_}]) ->
	NextId;
heuristic(local_search,_,{_DId,DPoint},ConPoints) ->
	DistFun = fun({From, P1}) -> {From, math:pow(DPoint#point.x-P1#point.x,2) + math:pow(DPoint#point.y-P1#point.y,2)} end,
	Dists = lists:map(DistFun,ConPoints),
	{NextId, _} = lists:foldl(fun({Id1,Dist}, {_Id,Min}) when Dist < Min -> {Id1,Dist}; (_,A) -> A end, {?undefined, ?inf}, Dists),
	NextId;
heuristic(randomized_distance_search,{CurId, CurPoint},{_DId,DPoint},ConPoints) ->
	DistFun = fun({From, P1}) -> {From, math:sqrt(math:pow(DPoint#point.x-P1#point.x,2) + math:pow(DPoint#point.y-P1#point.y,2))} end,
	{_,CurDist} = DistFun({CurId, CurPoint}),
	Dists = lists:map(DistFun,ConPoints),
	Diffs = [{Id,D - CurDist} || {Id,D} <- Dists],
	%io:format("diffs: ~w~n",[Diffs]),
	{AbsMax,_} = lists:max([{abs(X),Y} || {Y,X} <- Diffs]),
	Exps = [{Id,math:exp(D/AbsMax)} || {Id,D} <- Diffs],
	ExpSum = lists:sum([X || {_,X} <- Exps]),
	Heuristic = [{Id,E/ExpSum} || {Id,E} <- Exps],
	%io:format("heuristic: ~w~n",[Heuristic]),
	% accumulate values in list. e.g: [{a,0.1},{b,0.3},{c,0.2},{d,0.4}] -> [{a,0.1}, {b,0.4}, {c,0.6}, {d,1.0}]
	IntervalFun = fun(Old) -> Fun = fun(F, [{Id,H}|T], Sum, List) -> F(F,T,H+Sum,[{Id,H+Sum}|List]); (_,[],_,List) -> lists:reverse(List) end, Fun(Fun,Old,0,[]) end,
	% Draw a sample from 0.0 to N-1
	Sample = random:uniform(),% *(length(Heuristic)-1),
	% Check in which "interval" the sample resides
	Next = lists:nth(1,[Id || {Id,Upperbound} <- IntervalFun(Heuristic), Sample =< Upperbound]),
	%io:format("sample: ~w, ~w~n", [Sample,IntervalFun(Heuristic)]),
	%io:format("chosen: ~w~n", [Next]),
	Next.
	
create_shortest_path(#location{resource=Origin}, #location{resource=Destination}) ->
	modum_proxy:get_id() ! {get_shortest_path, Origin, Destination, self()},
	receive
		{?reply, get_shortest_path, Path} ->
			{D,_}=lists:foldl(fun(P,{D,K}) -> {dict:store(K,P,D),K+1} end, {dict:new(),0}, Path),D
	end.