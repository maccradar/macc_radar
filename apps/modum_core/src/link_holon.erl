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
-module(link_holon).
-behaviour(gen_server).

-export([start_link/1, stop/1, execution/2,print_history/3,get_history/3, get_description/1, get_road_type/1, get_state/1]).
-export([init/1, handle_call/3, handle_cast/2,
         handle_info/2, code_change/3, terminate/2]).

-include("states.hrl").
-include_lib("blackboard/include/blackboard_records.hrl").
-include_lib("util/include/debug.hrl").
-include_lib("eunit/include/eunit.hrl").

-include_lib("stdlib/include/ms_transform.hrl").

% the sample period, in [ms], for requesting map updates to the MODUM client.
-define(mapUpdateDelay, 300000).

% the sample period, in [ms], for propagating the traffic flow down the link.
-define(linkConstraintDelay,6000).

% the sample period, in [ms], for updating the cumulative flows in the blackboards.
-define(blackboardUpdateDelay,12000).
% the evaporation time, in [ms], of pheromones created by ants on the blackboards.
-define(evaporationTime,18000).

-define(deleteOldHistoryDelay, 30000).
-define(sumCumulativesDelay, 9000).
-define(historyWindow, {0,120,0}).

% start function of the gen_server, the link state representing this link holon has to be provided.
start_link(LinkState=#linkState{id=Id}) when is_atom(Id) ->
    gen_server:start_link({local,Id}, ?MODULE, [LinkState], []).

% access function to stop the link holon.
stop(Id) when is_atom(Id) ->
	util:log(info,{link, Id}, "stop is called", []),
	gen_server:call(Id, stop, ?callTimeout).

% initialisation of the link holon, based on its link state.
% - timers for the periodic updates are set
% - traffic flow blackboards are created
% - traffic time model and fundamental diagram is set up
% - output: initialized link being record
init([LinkState]) ->
	
    %% To know when the parent shuts down
	?CREATE_DEBUG_TABLE,
    process_flag(trap_exit, true),
    % util:log(info, {link, LinkState#linkState.id}, "Initialized with ~w #lanes and length ~w}~n", [LinkState#linkState.numLanes, LinkState#linkState.length]),
 	timer:send_interval(?mapUpdateDelay, updateMap),
 	timer:send_interval(?blackboardUpdateDelay, updateBlackboard),
    timer:send_interval(?linkConstraintDelay, propagateFlowDown),
	timer:send_interval(?deleteOldHistoryDelay, deleteOldHistory),
	timer:send_interval(?sumCumulativesDelay, sumCumulatives),
	%-record(linkBeing, {state=#linkState{},blackboard,models=#models{}}).
	BB_b = bb_trafficflow:create("BB_b_"++atom_to_list(LinkState#linkState.id)),
	BB_e = bb_trafficflow:create("BB_e_"++atom_to_list(LinkState#linkState.id)),
	BB_flow = bb_ets:create("BB_flow_"++atom_to_list(LinkState#linkState.id)),
	TTM = (fun(TimeBegin,LB) -> link_model:end_time(TimeBegin,LB) end),
	STM = (fun(TimeEnd,LB) -> link_model:start_time(TimeEnd,LB) end),
	FD = fundamental_diagram:init({city,LinkState#linkState.maxAllowedSpeed, LinkState#linkState.numLanes}),
	Flow= 0.0,% fundamental_diagram:q(fundamental_diagram:kc(FD), FD),
	M = #models{fd=FD, ttm=TTM, stm=STM},
	NewLinkState = LinkState#linkState{flow=Flow},
	LinkBeing = #linkBeing{state=NewLinkState,blackboard=#blackboard{bb_b=BB_b,bb_e=BB_e, bb_flow=BB_flow},models=M},
    init_ets_history(atom_to_list(LinkState#linkState.id), self()),
	{ok, LinkBeing}.


%%CF history ----------------------------------------------------------------------
%% -------------------------------------------------------------------------------- 
-record(history_item,{time,cf_begin,cf_end}).

get_history(png, LinkId, {Width, Height}) ->
	GetItem = fun(#history_item{time=_TimeStamp, cf_begin=CF_B, cf_end=CF_E},ACC)->
				[cumulative_func:to_png([{cf_b,CF_B},{cf_e,CF_E}], Width, Height) | ACC]
				end,
	Tab = list_to_atom("history_"++atom_to_list(LinkId)),
	case PNGS = lists:reverse(ets:foldl(GetItem, [], Tab)) of
		[] -> [cumulative_func:to_png([{cf_b_empty_history,void},{cf_e_empty_history,void}], Width, Height)];
		_ -> PNGS
	end;
get_history(points, LinkId, _Options) ->
	GetItem = fun(#history_item{time=_TimeStamp, cf_begin=CF_B, cf_end=CF_E},ACC)->
				[cumulative_func:cfs_to_points([{cf_b,CF_B},{cf_e,CF_E}], []) | ACC]
				end,
	Tab = list_to_atom("history_"++atom_to_list(LinkId)),
	case Points = lists:reverse(ets:foldl(GetItem, [], Tab)) of
		[] -> [[{cf_b_undefined,[{0,0}]},{cf_e_undefined,[{0,0}]}]];
		_ -> Points
	end.
print_history(png,LinkId,Dir)->
	PrintItem = fun(#history_item{time=TimeStamp,cf_begin=CF_B, cf_end=CF_E},ACC)->
							 File = Dir++ integer_to_list(TimeStamp)++".png",
							 cumulative_func:to_file(png,[{cf_b,CF_B},{cf_e,CF_E}],File),
							 ACC+1
							 end,
	Tab = list_to_atom("history_"++atom_to_list(LinkId)),
	ets:foldl(PrintItem, [], Tab);
	
print_history(text,LinkId,Dir)->
	PrintItem = fun(#history_item{time=TimeStamp,cf_begin=CF_B, cf_end=CF_E},ACC0)->
				   PrintCF = fun(CF,ACC)-> File = Dir++ integer_to_list(TimeStamp)++"_"++integer_to_list(ACC)++".txt",
										   cumulative_func:to_file(text,[CF],File),
										   ACC+1
							 end,
				   lists:foldl(PrintCF, 0, [{cf_b,CF_B},{cf_e,CF_E}]),														   
				   ACC0+1
				end,
	Tab = list_to_atom("history_"++atom_to_list(LinkId)),
	ets:foldr(PrintItem, 0, Tab).

init_ets_history(LinkId,_LinkPID)->
	TableName = list_to_atom("history_"++LinkId),
	ets:new(TableName, [public,named_table,ordered_set,{keypos,2}]).


%%-------------------------------------------------------------------------------------
%%------------------------------------------------------------------------------------- 

get_description(Id) ->
	{?reply, state, Info} = gen_server:call(Id,state,?callTimeout),
	Info#linkState.desc.

get_road_type(Id) ->
	{?reply, state, Info} = gen_server:call(Id,state,?callTimeout),
	Info#linkState.roadType.

get_state(Id) ->
	{?reply, state, Info} = gen_server:call(Id,state,?callTimeout),
	Info.
% callback for synchronous call to stop the link holon.
handle_call(stop, From, S) when is_record(S, linkBeing)->
	util:log(info,{link,(S#linkBeing.state)#linkState.id}, "handle call: stop from ~w",[From]),
    {stop, normal, From, S};
% callback for synchronous call to request the link state.
handle_call(state, _From, LB=#linkBeing{state=S}) ->
	{reply, {?reply, state, S}, LB};
handle_call(being, _From, LB=#linkBeing{}) ->
	{reply, {?reply, being, LB}, LB};
handle_call({flow_pheromone, Id}, _From, LB=#linkBeing{blackboard=#blackboard{bb_flow=BB}}) ->
	P = get_flow_pheromone(BB, Id),
	{reply, {?reply, flow_pheromone, P}, LB};
handle_call({travel_time, Times}, _From, LB=#linkBeing{state=#linkState{length=L, maxAllowedSpeed=S}, blackboard=#blackboard{cf_b=?undefined, cf_e=_CF_E}}) ->
	TravelTimes = [{T, round(L / S)} || T <- Times],
	{reply, TravelTimes, LB};
handle_call({travel_time, Times}, _From, LB=#linkBeing{state=#linkState{length=L, maxAllowedSpeed=S}, blackboard=#blackboard{cf_b=_CF_B, cf_e=?undefined}}) ->
	TravelTimes = [{T, 1+trunc(L / S)} || T <- Times],
	{reply, TravelTimes, LB};
handle_call({travel_time, Times}, _From, LB=#linkBeing{blackboard=#blackboard{cf_b=CF_B, cf_e=CF_E}}) when is_list(Times) ->
	TravelTimes = lists:map(fun(Time) -> {Time, 1+trunc(cumulative_flow:end_time(util:timestamp(sec,erlang:now())+Time,CF_B,CF_E))} end, Times),
	{reply, TravelTimes, LB};
% default callback for synchronous calls.
handle_call(_Message, From, S) ->
	util:log(error,{link, (S#linkBeing.state)#linkState.id}, "handle call from ~w",[From]),
    {noreply, S}.
handle_cast(traffic_update, LB) ->
	NewLB = get_traffic_update(LB),
    {noreply, NewLB};
handle_cast({updateBeing,B}, _LB) ->
	{noreply, B};
% default callback for asynchronous casts.
handle_cast(_Message, S) ->
	util:log(error,{link, (S#linkBeing.state)#linkState.id}, "handle cast: state =  ~w",[S]),
    {noreply, S}.
% callback to handle updateMap message. This is periodically called to request map updates through the MODUM client.
handle_info(updateMap, LB) ->
    NewLB = get_traffic_update(LB),
	{noreply, NewLB};
handle_info(deleteOldHistory, LB=#linkBeing{state=#linkState{id=Id}}) ->
	Now = util:timestamp(sec,erlang:now()),
	Window = util:timestamp(sec,?historyWindow),
	_Nbr = ets:select_delete(list_to_atom("history_"++atom_to_list(Id)), ets:fun2ms(fun(#history_item{time=Time}) when Time < Now-Window -> true end)),
	{noreply, LB};

handle_info(sumCumulatives, LB=#linkBeing{state=#linkState{id=Id}, blackboard=BB}) ->
	spawn(
		fun() ->
			BB_Flow = BB#blackboard.bb_flow,
			CFs = get_flow_cfs(BB_Flow),
			CFs == [] orelse util:log(debug,{link, Id},"Summing CFs: ~w", [[cumulative_func:cf_to_points(C) || C <- CFs]]),
			NewCF_Flow = cumulative_func:sum(CFs),
			NewCF_Flow == [] orelse util:log(debug,{link, Id}, "New CF_Flow: ~w", [cumulative_func:cf_to_points(NewCF_Flow)]),
			NewBB = BB#blackboard{cf_flow=NewCF_Flow},
			NewLB = LB#linkBeing{blackboard=NewBB},
			gen_server:cast(Id,{updateBeing, NewLB})
		end
	),
	{noreply, LB};
	
% callback to handle updateBlackboard message. This is periodically called to update the cumulative flows, maintained by the blackboards.
handle_info(updateBlackboard, #linkBeing{blackboard=BB} = LB)->
	bb_trafficflow:request_cumulative(BB#blackboard.bb_b,bb_b),
	% bb_trafficflow:request_cumulative(BB#blackboard.bb_e,bb_e),
	{noreply, LB};

% callback to handle updateBlackboard message. This is the reply from the begin blackboard with the new cumulative flow
handle_info({updateBlackboard,bb_b,CF_B_points}, #linkBeing{state=#linkState{id=ID},blackboard=BB=#blackboard{cf_flow=CF_Flow}} = LB)->
	CF_B = cumulative_flow:get_cumulative(CF_B_points),
	CF_Flow == [] orelse util:log(debug,{link, ID}, "Summing CF_B ~w and CF_Flow ~w", [CF_B, cumulative_func:cf_to_points(CF_Flow)]),
	SumCF = cumulative_func:sum(CF_B, CF_Flow),
	SumCF == ?undefined orelse util:log(debug,{link, ID}, "New CF_B: ~w", [cumulative_func:cf_to_points(SumCF)]),
	NewLB = LB#linkBeing{blackboard=BB#blackboard{cf_b=SumCF}},
	ets:insert(list_to_atom("history_"++atom_to_list(ID)), #history_item{time=util:timestamp(sec,erlang:now()),cf_end=BB#blackboard.cf_e,cf_begin=CF_B}),
	{noreply, NewLB};
% callback to handle updateBlackboard message. This is the reply from the end blackboard with the new cumulative flow
% handle_info({updateBlackboard,bb_e,CF_E_points}, #linkBeing{state=#linkState{id=ID},blackboard=BB} = LB)->
	% CF_E = cumulative_flow:get_cumulative(CF_E_points),
	% NewCF_E = case {CF_E, BB#blackboard.cf_e} of
		% {?undefined, CF2} -> CF2;
		% {CF1, ?undefined} -> CF1;
		% {CF1, _CF2} -> CF1 % TODO: what do we do here? just replace it?
	% end,
	% NewLB = LB#linkBeing{blackboard=BB#blackboard{cf_e=NewCF_E}},
	% ets:insert(list_to_atom("history_"++atom_to_list(ID)), #history_item{time=util:timestamp(sec,erlang:now()),cf_begin=BB#blackboard.cf_b,cf_end=NewCF_E}),
	% {noreply, NewLB};
% callback to handle propagateFlowDown message. This is periodically called to propagate traffic flow down the link and thereby update the cumulative flows.
handle_info(propagateFlowDown, LB =  #linkBeing{state=#linkState{id=ID},blackboard=BB,models=#models{fd=FD} }) ->
	spawn(
		fun() ->
			% only propagateFlowDown if CF_B and CF_E is not undefined
			case {BB#blackboard.cf_b,BB#blackboard.cf_e } of
				{CF_B, _CF_E} when (CF_B == ?undefined) -> void;
							 % io:format("CF_B undefined. not propagating flow down~n"),
							 % {noreply, LB};
				{CF_B, _CF_E}->MaxGrad = fundamental_diagram:c(FD),
		%%					  CF_B1 =  link_model:accommodate_max_capacity( CF_B, MaxGrad),
							  CF_E1 = link_model:propagate_flow(down,CF_B, LB),
		%% 					  ets:insert(list_to_atom("history_"++atom_to_list(ID)), #history_item{time=util:timestamp(sec,erlang:now()),cf_begin=CF_B1,cf_end=CF_E}),
		%%					  NewLB1 = LB#linkBeing{blackboard=BB#blackboard{cf_b=CF_B1,cf_e=CF_E1}},  
		%%					  UPContraint = link_model:propagate_flow(up,CF_E1, NewLB1),
		%% 					  ets:insert(list_to_atom("history_"++atom_to_list(ID)), #history_item{time=util:timestamp(sec,erlang:now()),cf_begin=CF_B1,cf_end=UPContraint}),
		%%					  NewCF_B = cumulative_flow:constrain_cf(CF_B1, UPContraint),
							  CF_E2 = case link_model:accommodate_max_capacity(CF_E1, MaxGrad) of
								illegal_state -> util:log(error,{link, ID}, "Illegal state in accommodate max capacity for ~w, ~w", [cumulative_func:cf_to_points(CF_E1), MaxGrad]), CF_E1;
								NewCF_E -> NewCF_E
							  end,
		%% 					  ets:insert(list_to_atom("history_"++atom_to_list(ID)), #history_item{time=util:timestamp(sec,erlang:now()),cf_begin=CF_B1,cf_end=NewCF_B}),
							  util:log(debug,{link, ID}, "New CF_E: ~w", [cumulative_func:cf_to_points(CF_E2)]),
							  NewLB2 = LB#linkBeing{blackboard=BB#blackboard{cf_b=CF_B,cf_e=CF_E2}},
		%%					  ets:insert(list_to_atom("history_"++atom_to_list(ID)), #history_item{time=util:timestamp(sec,erlang:now()),cf_begin=NewCF_B,cf_end=CF_E}),
							  ID ! {updateBeing, NewLB2}
			end
		end
	),
	{noreply,LB};
handle_info({updateBeing,B = #linkBeing{blackboard=#blackboard{cf_b=_CF_B, cf_e=_CF_E, cf_flow=_CF_Flow}}}, _LB) ->
	% io:format("Link being update: ~w~n", [{CF_B,CF_E,CF_Flow}]),
	{noreply, B};
% callback to handle state message. This is used to request the link state record.
% the caller's Pid or registered name is required in the message to be able to send the reply.
handle_info({state, Pid}, LB=#linkBeing{state=S}) ->
	% io:format("Link receive state request.~n"),
	Pid ! {?reply, state, S},
	{noreply, LB};
% callback to handle blackboard message. This is used to request the link blackboard record, containing both blackboards and cumulative flows.
% the caller's Pid or registered name is required in the message to be able to send the reply.
handle_info({blackboard, Pid}, LB=#linkBeing{blackboard=B}) ->
	Pid ! {?reply, blackboard, B},
	{noreply, LB};
% callback to handle being message. This is used to request the link being record.
% the caller's Pid or registered name is required in the message to be able to send the reply.
handle_info({being, Pid}, LB) ->
	Pid ! {?reply, being, LB},
	{noreply, LB};
handle_info({upstream_connections,Pid},LB=#linkBeing{state=#linkState{connection = Connection}})->
	Pid ! {?reply, upstream_connections, [Connection#connection.from]},
	{noreply, LB};
handle_info({downstream_connections,Pid},LB = #linkBeing{state=#linkState{connection = Connection}})->
	Pid ! {?reply, downstream_connections, [Connection#connection.to]},
	{noreply, LB};
handle_info({downstream_connections,From,Pid},LB = #linkBeing{state=#linkState{connection = #connection{from=From,to=To}}}) ->
	Pid ! {?reply, downstream_connections, [To]},
	{noreply, LB};
handle_info({downstream_connections,_,Pid},LB) ->
	Pid ! {?reply, downstream_connections, []},
	{noreply, LB};
handle_info({downstream_point, Pid}, LB = #linkBeing{state=#linkState{shape=Shape}}) ->
	[Last | _] = lists:reverse(Shape),
	Pid ! {?reply, downstream_point, Last},
	{noreply, LB};
handle_info({upstream_point, Pid}, LB = #linkBeing{state=#linkState{shape=Shape}}) ->
	[First | _] = Shape,
	Pid ! {?reply, upstream_point, First},
	{noreply, LB};
handle_info({'EXIT', _Pid, Reason}, State) ->
	util:log(error,{link,(State#linkBeing.state)#linkState.id}, "Exit received with reason ~p", Reason),
    {stop, normal, Reason, State};
% callback to handle scenario message. This is used to request the execution of a scenario on the link holon.
% input: {ExecutionType, Scenario, SenderId}
% TODO add messagetag (avoid matching other messages)
handle_info(Message = {_,_Scenario,_SenderId}, LB) ->
	execution(Message,LB),
	{noreply, LB};
handle_info({subscribe, get_info, Time, once, Receiver}, LB) ->
	timer:send_after(Time, {get_info, Receiver}),
	{noreply, LB};
handle_info({subscribe, get_info, Interval, repeat, Receiver}, LB) ->
	timer:send_interval(Interval, {get_info, Receiver}),
	{noreply, LB};
handle_info({subscribe, get_density, Interval, repeat, Receiver}, LB) ->
	timer:send_interval(Interval, {get_density, Receiver}),
	{noreply, LB};
handle_info({get_info, Pid},  LB=#linkBeing{state=#linkState{id=Id}}) ->
	Pid ! {info, Id, LB},
	{noreply, LB};
handle_info({get_density, 0, discrete, Pid}, LB=#linkBeing{models=#models{fd=FD},state=#linkState{id=Id, density=Density, coordinates=Coordinates}}) ->
	util:log(info,{link,Id},"Density: ~w QoS: ~w", [Density, density_to_level_of_service(Density, fundamental_diagram:kjam(FD))]),
	
	Pid ! {density, Id, {Coordinates,density_to_level_of_service(Density, fundamental_diagram:kjam(FD))}},
	{noreply, LB};
handle_info({get_density, Time, discrete, Pid}, LB=#linkBeing{models=#models{fd=FD}, blackboard=#blackboard{cf_b = CF_B, cf_e = CF_E}, state=#linkState{length=L, id=Id, density=_Density, coordinates=Coordinates}}) when L /= 0 ->
	Now = util:timestamp(sec,erlang:now()),
	N = cumulative_flow:nbr_vehicles(Time+Now,CF_B,CF_E),
	QoS = case L < ?vehicle_length of
		true ->	density_to_level_of_service(0,fundamental_diagram:kjam(FD));
		_ -> density_to_level_of_service(N/L,fundamental_diagram:kjam(FD))
	end,
	((QoS < 0) or (N < 0)) andalso util:log(error,{link,Id},"QoS: ~w. #vehicles: ~w at ~w, CF_B: ~w, CF_E: ~w",[QoS,N,Time+Now, cumulative_func:cf_to_points(CF_B),cumulative_func:cf_to_points(CF_E)]),
	Pid ! {density, Id, {Coordinates,QoS}},
	{noreply, LB};
% default callback for messages.
handle_info(Message, S) ->
	util:log(error,{link, (S#linkBeing.state)#linkState.id}, "Unknown message received: ~w", [Message]),
    {noreply, S}.

% default code_change callback.
code_change(_OldVsn, State, _Extra) ->
    {ok, State}.

% default terminate callbacks.
terminate(normal, S) ->
    util:log({link, (S#linkBeing.state)#linkState.id}, "Normal termination~n",[]);
terminate(shutdown, S) ->
    util:log({link, (S#linkBeing.state)#linkState.id}, "Shutdown termination", []);
terminate(Reason, S) ->
    util:log({link, (S#linkBeing.state)#linkState.id}, "Killed with reason ~p", [Reason]).

%%%%%%%%%%%%%%%%%%%%%%
% Internal Functions %
%%%%%%%%%%%%%%%%%%%%%%

density_to_level_of_service(Density, KJam) ->
	round(100*(1 - Density / KJam)).
	
%% execution of models
execution({execute, #scenario{timeSlot={Time,_}}, SenderId}, LB=#linkBeing{state=#linkState{id=_Id},models=M}) ->
    ET =(M#models.ttm)(Time,LB),
	case ET of
		ET when ((is_number(ET)) and (ET > Time)) -> SenderId ! {?reply,execute, ET};
		T -> io:format("end time is not calculated correctly ~w~n",[T])
	end;
execution({proclaim, #scenario{timeSlot={Time,_},antState=#antState{creatorId=VehicleId}},SenderId}, LB=#linkBeing{state=#linkState{id=_Id},blackboard=B,models=M}) ->
	% add arrival time to blackboard
 	pheromone:create([B#blackboard.bb_b], ?evaporationTime,#vehicle_info{vehicle=VehicleId, arrival_time=Time,data=void}),
	% apply travel time model  
	ET =(M#models.ttm)(Time,LB),
	case ET of
		ET when ((is_number(ET)) and (ET > Time)) -> % pheromone:create([B#blackboard.bb_e], ?evaporationTime,#vehicle_info{vehicle=VehicleId, arrival_time=ET,data=void}),
								  SenderId ! {?reply,proclaim, ET};
		T -> io:format("end time is not calculated correctly ~w~n",[T])
	end;
execution({explore_upstream, #scenario{timeSlot={_,Time}}, SenderId}, LB=#linkBeing{state=#linkState{id=Id},models=M}) ->
    ET =(M#models.stm)(Time,LB),
	case ET of
		ET when ((is_number(ET)) and (ET < Time)) -> 
			io:format("start time for link ~w: ~w~n",[Id,ET]),
			SenderId ! {?reply,explore_upstream, ET};
		T -> io:format("start time is not calculated correctly ~w~n",[T])
	end;
execution({proclaim_flow, #scenario{antState=#antState{location=Location, creatorId=Id, data=CF}}, SenderId}, LB=#linkBeing{blackboard=#blackboard{bb_flow=BB},state=#linkState{id=LinkId,connection = Connection}}) ->
	% add new {linkId, cumulative function} tuple to blackboard
    % util:log({link, LinkId}, "Adding pheromone ~w", [cumulative_func:cf_to_points(CF)]),
	pheromone:create([BB], ?evaporationTime, #info{data={Id, CF}, tags=[flow]}),
	% propagate flow down 
	NewCF = link_model:propagate_flow(down,CF, LB),
	% io:format("New CF in link ~w: ~w~n", [LinkId,{Connection#connection.to, cumulative_func:cfs_to_points([{LinkId, NewCF}],[])}]),
	% send new cumulative function back to current flow ant
	% io:format("creating current flow ant on ~w for ~w~n", [Connection#connection.to, Id]),
	spawn(
		fun() ->
			traffic_ant:create_current_flow_ant(Location, Connection#connection.to, Id, NewCF)
		end
	).
	% SenderId ! {?reply, proclaim_flow, [{Connection#connection.to, NewCF}]}.

get_flow_cfs(Blackboard) ->
	Blackboard ! {get,{'_',{'_', '$1'},'_'},self()},
	receive
		{result_get,[]}->[];
		{result_get,CFs} -> lists:flatten(CFs);
		M -> io:format("Received unknown message expected flow result: ~w~n", [M])
	after 5000-> io:format("Waiting too long for flow response...~n"), []
	end.
get_flow_pheromone(Blackboard, Id) ->
	Blackboard ! {get,{'$1',{Id, '_'},'_'},self()},
	receive
		{result_get,[]}->?undefined;
		{result_get,[[PheromoneId]]} -> PheromoneId;
		M -> io:format("Received unknown message expected pheromone result: ~w~n", [M])
		after 5000-> io:format("Waiting too long for pheromone response...~n"), ?undefined
	end.

get_traffic_update(LB=#linkBeing{state=L, models=#models{fd=FD}}) ->
	{?reply, linkUpdate, LinkState} = gen_server:call(modum_proxy:get_id(),{linkUpdate, L}, ?callTimeout),
	% inform downstream node of "new" capacity
	ToNode = (LinkState#linkState.connection)#connection.to,
	Capacity = fundamental_diagram:c(FD),
	ToNode ! {capacityUpdate, LinkState#linkState.id, Capacity},
	NewLB = LB#linkBeing{state=LinkState},
	% send current flow ant
	% TimeToPass = LinkState#linkState.length / LinkState#linkState.maxAllowedSpeed,
	% Flow= fundamental_diagram:q(fundamental_diagram:kc(FD), FD),
	TimeToPass = case LinkState#linkState.avgSpeed of
		0.0 -> LinkState#linkState.length / 0.01;
		Speed -> LinkState#linkState.length / Speed
	end,
	TimeInterval = 300, % 5 minutes of sampling
	Flow = LinkState#linkState.flow,
	% io:format("Flow: ~w, density: ~w for link ~w~n", [Flow, LinkState#linkState.density, LinkState#linkState.id]),
	% io:format("Capacity: ~w, Flow: ~w~n", [Capacity,Flow]),
	NumberOfVehicles = round(TimeInterval * Flow),
	Now = util:timestamp(sec,erlang:now()),
	CumulativeFunction = cumulative_func:add_point(TimeToPass+Now,NumberOfVehicles,cumulative_func:new(Now,0)),
	NumberOfVehicles > 0 andalso traffic_ant:create_current_flow_ant(#location{resource=LinkState#linkState.id}, ToNode, LinkState#linkState.id, CumulativeFunction),
	NewLB.
