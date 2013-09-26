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
-module(node_holon).
-behaviour(gen_server).

-export([start_link/1, stop/1, get_description/1, get_coordinates/1, get_distance/2, get_turning_fractions/1]).
-export([init/1, handle_call/3, handle_cast/2,
         handle_info/2, code_change/3, terminate/2]).
-include("states.hrl").
-define(DELAY, 300000).

get_description(Id) ->
	{?reply, state, S} = gen_server:call(Id, state, ?callTimeout),
	S#nodeState.desc.

get_coordinates(Id) ->
	{?reply, state, S} = gen_server:call(Id, state, ?callTimeout),
	S#nodeState.coordinates.

get_distance(Id, {Lat1,Lon1}) ->
	{?reply, state, S} = gen_server:call(Id, state, ?callTimeout),
	[{Lat2,Lon2}] = S#nodeState.coordinates,
	math:pow(Lat1-Lat2,2) + math:pow(Lon1-Lon2,2).
	
get_turning_fractions(Id) ->
	{?reply, turning_fractions, TF} = gen_server:call(Id, turning_fractions, ?callTimeout),
	TF.
	
start_link(NodeState=#nodeState{id=Id}) when is_atom(Id) ->
    gen_server:start_link({local,Id}, ?MODULE, [NodeState], []).

stop(Id) when is_atom(Id) -> 
	gen_server:call(Id, stop, ?callTimeout).

init([NodeState]) ->
    %% To know when the parent shuts down
    process_flag(trap_exit, true),
    %io:format("Initialized node with state {~s,~s}~n", [NodeState#nodeState.id, NodeState#nodeState.desc]),
	TTM = (fun(TimeBegin) -> TimeBegin end),
	STM = (fun(TimeEnd) -> TimeEnd end),
	FD = fundamental_diagram:init(0.015, 0.5, 0.2), 
	M = #models{fd=FD, ttm=TTM, stm=STM},
	NodeBeing = #nodeBeing{state=NodeState#nodeState{capacities=dict:new(), turningFractions=dict:new()},models=M},
	{ok, NodeBeing}.

handle_call(check_consistency, _From, NB=#nodeBeing{state=#nodeState{id = Id,turningFractions=TF, connections = Connections}}) ->
	CheckFun = fun(#connection{from=From, to=To}) ->
		Ok = dict:is_key({From,To},TF),
		Ok orelse io:format("check failed for node ~w on link pair: ~w, ~w~n",[Id,From,To]),
		Ok
	end,
	Result = lists:all(CheckFun, Connections),
	{reply, {?reply, check_consistency, Result}, NB};

handle_call(turning_fractions, _From, S=#nodeBeing{state=#nodeState{turningFractions=TF}}) ->
	{reply, {?reply, turning_fractions, dict:to_list(TF)}, S};
handle_call(connections, _From, S=#nodeBeing{state=#nodeState{connections=C}}) ->
	{reply, {?reply, connections, C}, S};
handle_call(state, _From, NB=#nodeBeing{state=S}) ->
	{reply, {?reply, state, S}, NB};
handle_call(stop, _From, S=#nodeBeing{}) ->
    {stop, normal, ok, S};
handle_call(_Message, _From, S) ->
    {noreply, S}.

handle_cast(_Message, S) ->
    {noreply, S}.

handle_info({upstream_connections,Pid},NB=#nodeBeing{state=#nodeState{connections = Connections}})->
	UpstreamConnections = lists:foldl(fun(#connection{from = Connection},Connections2)-> [Connection|Connections2] end, [], Connections),
	Pid ! {?reply, upstream_connections, UpstreamConnections},
	{noreply, NB};
handle_info({downstream_connections,Pid},NB = #nodeBeing{state=#nodeState{connections = Connections}})->
	DownstreamConnections = lists:foldl(fun(#connection{to = Connection},Connections2)-> [Connection|Connections2] end, [], Connections),
	Pid ! {?reply, downstream_connections, DownstreamConnections},
	{noreply, NB};
handle_info({downstream_connections, From, Pid},NB = #nodeBeing{state=#nodeState{connections = Connections}})->
	%io:format("Connections from ~w:~w~n",[From,Connections]),
	DownstreamConnections = lists:foldl(fun(#connection{from = FromC, to = ToC},Connections2) when FromC == From -> [ToC|Connections2]; (_,Connections2) -> Connections2 end, [], Connections),
	%io:format("downstream connections: ~w~n",[DownstreamConnections]),
	Pid ! {?reply, downstream_connections, DownstreamConnections},
	{noreply, NB};
handle_info({downstream_point,Pid}, NB=#nodeBeing{state=#nodeState{shape=Shape}})->
	[Last | _] = lists:reverse(Shape),
	Pid ! {?reply, downstream_point, Last},
	{noreply, NB};
handle_info({upstream_point,Pid}, NB=#nodeBeing{state=#nodeState{shape=Shape}})->
	[First | _] = Shape,
	Pid ! {?reply, upstream_point, First},
	{noreply, NB};
handle_info(Message = {execute,_Scenario,_SenderId}, NB) ->
	execution(Message,NB),
	{noreply, NB};
handle_info(Message = {proclaim,_Scenario,_SenderId}, NB) ->
	execution(Message,NB),
	{noreply, NB};
handle_info(Message = {explore_upstream,_Scenario,_SenderId}, NB) ->
	execution(Message,NB),
	{noreply, NB};
handle_info({being, Pid}, NB) ->
	Pid ! {?reply, being, NB},
	{noreply, NB};
handle_info({state, Pid}, NB=#nodeBeing{state=S}) ->
	Pid ! {?reply, state, S},
	{noreply, NB};
handle_info({capacityUpdate,LinkId,Capacity}, NB=#nodeBeing{state=NS=#nodeState{capacities=C}}) ->
	%io:format("Updating link capacity value: {~w, ~w}~n",[LinkId, Capacity]),
	NewC = dict:store(LinkId, Capacity, C),
	NewNB = NB#nodeBeing{state=NS#nodeState{capacities=NewC}},
	{noreply, NewNB};
handle_info({subscribe, get_info, Time, once, Receiver}, NB) ->
	timer:send_after(Time, {get_info, Receiver}),
	{noreply, NB};
handle_info({subscribe, get_info, Interval, repeat, Receiver}, NB) ->
	timer:send_interval(Interval, {get_info, Receiver}),
	{noreply, NB};
handle_info({get_info, Pid},  NB=#nodeBeing{state=#nodeState{id=Id}}) ->
	Pid ! {info, Id, NB},
	{noreply, NB};
handle_info({add_turning_fractions, FromId, Tos}, NB=#nodeBeing{state=NS=#nodeState{turningFractions=TF, connections=Connections}}) ->
	ToFun = fun({ToId, Prob}, AccTF) ->
		dict:store({FromId,ToId}, Prob, AccTF)
		end,
	NewTF = lists:foldl(ToFun, TF, Tos),
	NewNB = NB#nodeBeing{state=NS#nodeState{turningFractions=NewTF}},
	{noreply, NewNB};
handle_info(Message, S) ->
	io:format("Node ~s received unknown message ~w~n",[(S#nodeBeing.state)#nodeState.id, Message]),
    {noreply, S}.

code_change(_OldVsn, State, _Extra) ->
    {ok, State}.

terminate(normal, S) ->
    io:format("Node ~s terminated normally~n",[(S#nodeBeing.state)#nodeState.id]);
terminate(shutdown, S) ->
    io:format("Node ~s got shutdown~n",[(S#nodeBeing.state)#nodeState.id]);
terminate(Reason, S) ->
    io:format("Node ~w got killed with reason: ~w~n", [(S#nodeBeing.state)#nodeState.id, Reason]).


%% execution of models
execution({execute, #scenario{timeSlot={Time,_}}, SenderId}, #nodeBeing{models=M, state=#nodeState{id=_Id}}) ->
 	%io:format("Holon ~w executes execute call.~n", [Id]),
	ET = (M#models.ttm)(Time),
 	SenderId ! {?reply,execute, ET};
execution({proclaim, #scenario{timeSlot={Time,_}},SenderId}, #nodeBeing{models=M, state=#nodeState{id=_Id}}) ->
 	%io:format("Holon ~w executes proclaim call.~n", [Id]),
 	ET = (M#models.ttm)(Time),
	SenderId ! {?reply,proclaim, ET};
execution({explore_upstream, #scenario{timeSlot={_,Time}}, SenderId}, #nodeBeing{state=#nodeState{id=Id},models=M}) ->
    ET =(M#models.stm)(Time),
	case ET of
		ET when ((is_number(ET)) and (ET =< Time)) -> 
			io:format("start time for node ~w: ~w~n",[Id,ET]),
			SenderId ! {?reply,explore_upstream, ET};
		T -> io:format("start time is not calculated correctly ~w~n",[T])
	end.
