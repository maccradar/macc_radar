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
-module(modum_proxy).
-behaviour(gen_server).

-export([start_link/1, stop/0, create_graph/1, vertex/1, get_forecast/1, get_graph/0, get_k_shortest_path/3, get_status_info/0, get_server/1, get_client/1, get_links_of_type/1, get_closest_node/1]).
-export([init/1, handle_call/3, handle_cast/2,
         handle_info/2, code_change/3, terminate/2, get_id/0]).

-include("states.hrl").
-include("xml.hrl").

% the sample period, in [ms], for requesting updates to the UTMC server.
-define(DELAY, 300000).
% the default id, i.e. registered name, of the modum client module.
-define(ID, modum_proxy).

create_graph(Data) ->
	get_id() ! {create_graph, Data}.

vertex(V) ->
	get_id() ! {vertex, V, self()},
	receive
		{?reply, vertex, V1} -> V1
	end.

get_forecast(TimeWindow) ->
	gen_server:call(?ID, {forecast, TimeWindow}, ?callTimeout).
	
get_closest_node({Lat,Lon}) ->
	{?reply, closest_node, NodeId} = gen_server:call(?ID, {closest_node, {Lat,Lon}}, ?callTimeout),
	NodeId.
	
get_graph() ->
	get_id() ! {get_graph, self()},
	receive
		{?reply, get_graph, G} -> G
	end.

get_links_of_type(RoadType) ->
	{?reply, links_of_type, LinkIds} = gen_server:call(?ID, {links_of_type, RoadType}, ?callTimeout),
	LinkIds.
	
get_k_shortest_path(Begin, End, K) ->
	get_id() ! {get_k_shortest_path, Begin, End, K, self()},
	receive
		{?reply, get_k_shortest_path, Ps} -> Ps
	end.

get_status_info() ->
	{?reply, status_info, Info} = gen_server:call(?ID, status_info, ?callTimeout),
	Info.
get_client(Name) ->
	{?reply, client, Client} = gen_server:call(?ID, {client,Name},?callTimeout),
	Client.
get_server(Name) ->
	{?reply, server, Server} = gen_server:call(?ID, {server,Name},?callTimeout),
	Server.
% returns the id of the modum client which can be used to send message to.
get_id() ->
	?ID.

% start function of the gen_server, the model used to interprete the xml messages has to be provided.
start_link(ProxyState=#proxyState{}) ->
	gen_server:start_link({local, ?ID}, ?MODULE, [ProxyState], []).

% access function to stop the modum_client.
stop() -> gen_server:call(?ID, stop, ?callTimeout).

% initialisation of the modum client. this function is automatically called by the gen_server:start_link function.
% -input: the xml model
% -output: state used in the gen_server, here the #proxyState record is used.
init([ProxyState]) ->
	%% To know when the parent shuts down
    process_flag(trap_exit, true),
	%{ok, NodeDict, LinkDict} = parseMap(Map),
	%{ok, Graph} = createGraph(Map),
	% start by requesting a traffic update 10 seconds after boot
	timer:send_after(10000, traffic_update),
	timer:send_interval(?DELAY, traffic_update),
	{ok, ProxyState#proxyState{nodeInfoDict=dict:new(), linkInfoDict=dict:new(), cache=dict:new()}}.
	
% callback for synchronous call to stop the modum client.
handle_call(stop, _From, S=#proxyState{}) ->
    {stop, normal, ok, S};
% callback for synchronous call to get system time
handle_call(time, _From, S) ->
	{reply,{?reply, time, util:timestamp(sec,erlang:now())}, S};
% callback for synchronous call to request a node update.
% this function replies with the most recent node state data, which is retrieved in the node info dictionary.
handle_call({nodeUpdate, NodeState=#nodeState{id=NodeId}}, _From, S=#proxyState{nodeInfoDict=NodeDict}) ->
	%io:format("Node ~w requests update through call.~n", [NodeId]),
	case dict:find(NodeId, NodeDict) of
		{ok, _NodeInfo} -> % TODO: parse nodeInformationType (does not exist yet)
			{reply, {?reply, nodeUpdate, NodeState}, S};
		error -> % no updates received, so reply with current state
			{reply, {?reply, nodeUpdate, NodeState}, S}
	end;
% callback for synchronous call to request a link update.
% this function replies with the most recent link state data, which is retrieved in the link info dictionary.
handle_call({linkUpdate, LinkState=#linkState{id=LinkId}}, _From, S=#proxyState{linkInfoDict=LinkDict}) ->
	%io:format("Link ~w requests update through call.~n", [LinkId]),
	case dict:find(LinkId, LinkDict) of
		{ok, #linkInformationType{avgSpeed=AvgSpeed,co2emissions=CO2,density=Density, flow=Flow}} ->
			NewLinkState = LinkState#linkState{avgSpeed=list_to_float(AvgSpeed), co2emissions=list_to_float(CO2), density=list_to_float(Density), flow=list_to_float(Flow)},
			% io:format("Recent info found, replying with new state~n"),
			{reply, {?reply, linkUpdate, NewLinkState}, S};
		error -> % no updates received, so reply with current state
			% io:format("No recent info found, replying with current state~n"),
			{reply, {?reply, linkUpdate, LinkState}, S}
	end;

handle_call(status_info, _From, S=#proxyState{nodes=N,links=L}) ->
	{reply, {?reply, status_info, {L,N}},S};

handle_call({server,Name}, _From, State=#proxyState{servers=Servers}) ->
	[Server | _] = [S || {N,S} <- Servers, N == Name],
	{reply, {?reply, server, Server},State};
handle_call({client,Name}, _From, State=#proxyState{clients=Clients}) ->
	[Client | _] = [C || {N,C} <- Clients, N == Name],
	{reply, {?reply, client, Client},State};

handle_call({links_of_type,RoadType}, _From, State=#proxyState{cache=Cache, links=Links}) ->
	case dict:find({link_ids, RoadType},Cache) of
		{ok, LinkIds} ->  
			{reply, {?reply, links_of_type, LinkIds}, State};
		error -> 
			Result = [LinkId || LinkId <- Links, link_holon:get_road_type(LinkId) == RoadType],
			NewCache = dict:store({link_ids, RoadType}, Result, Cache),
			{reply, {?reply, links_of_type, Result}, State#proxyState{cache=NewCache}}
	end;

handle_call({closest_node, {Lat,Lon}}, _From, State=#proxyState{nodes=Nodes}) ->
	Dists = [{NodeId, node_holon:get_distance(NodeId, {Lat,Lon})} || NodeId <- Nodes],
	{ClosestId, _Distance} = lists:foldl(fun({Id1,Dist}, {_Id,Min}) when Dist < Min -> {Id1,Dist}; (_,A) -> A end, {?undefined, ?inf}, Dists),
	{reply, {?reply, closest_node, ClosestId}, State};

handle_call({check_consistency, nodes}, _From, State = #proxyState{nodes=Nodes}) ->
	CheckFun = fun(NodeId) ->
		{?reply, check_consistency, Check} = gen_server:call(NodeId,check_consistency),
		Check
	end,
	Result = lists:all(CheckFun, Nodes),
	{reply, {?reply, {check_consistency, nodes}, Result}, State};

handle_call({forecast, TimeWindow}, _From, State = #proxyState{links=Links}) ->
	TimeStep = 300,
	Times = lists:seq(0, TimeWindow, TimeStep), % sample every 5 minutes
	F = fun(LinkId) ->
		{LinkId, gen_server:call(LinkId, {travel_time, Times}, ?callTimeout), TimeStep}
	end,
	Forecast = lists:map(F, Links),
	{reply, Forecast, State};
% default callback for synchronous calls.
handle_call(_Message, _From, S) ->
    {noreply, S}.
handle_cast({traffic_update_response, Response}, S=#proxyState{linkInfoDict=Dict}) ->
	NewDict = case Response of
		?undefined -> Dict;
		_ -> 	io:format("Received parsed response, updating link states~n"),
				updateLinkStates(Dict, Response#updateResponse.map#mapInformationType.link)
	end,
	NewS = S#proxyState{linkInfoDict=NewDict},
    {noreply, NewS};
% default callback for casts.
handle_cast(_Message, S) ->
    {noreply, S}.

% callback to handle the interval message. This is used to trigger an update request to the UTMC server.
handle_info(traffic_update, S = #proxyState{model=Model, clients=Clients}) ->
    [TUC | _] = [C || {N,C} <- Clients, N == ?traffic_update_client],
	%{transmit, Response} = 
	gen_server:cast(TUC,{transmit, {async, Model}}),
    {noreply, S};
	
% callback to handle the state message. This can be used to request the state of the modum client.
% the sender pid has to be specified in the message.
handle_info({state, Pid}, S=#proxyState{}) ->
	Pid ! {?reply, state, S},
	{noreply, S};
% callback to handle the create_graph message. This can be used to create a digraph based on the topology of the nodes and links.
% node states and link states have to be specified in the message. The created graph has link and node id's (registered names or pids)
% as vertices and edges between connected nodes and links.
handle_info({create_graph, {NodeStates, LinkStates}}, S=#proxyState{}) ->
	{ok, {G,N,L}} = createGraph({NodeStates, LinkStates}),
	NewS = S#proxyState{graph = G,nodes=N, links=L},
	{noreply, NewS};
% callback to handle the get_graph message. This can be used to request the digraph.
% the sender pid has to be specified in the message.
handle_info({get_graph, Pid}, S=#proxyState{graph=G}) ->
	Pid ! {?reply, get_graph, G},
	{noreply, S};
% callback to handle the get_shortest_path message. This can be used to request the shortest path between two vertices in the digraph.
% begin vertex, end vertex and sender pid have to be specified in the message. The returned path is a list of vertices.
handle_info({get_shortest_path, Begin, End, Pid}, S=#proxyState{graph=G}) ->
	Path = astar:astar(G, Begin, End),
	Pid ! {?reply, get_shortest_path, Path},
	{noreply, S};
handle_info({get_k_shortest_path, Begin, End, K, Pid}, S=#proxyState{graph=G, cache=Cache}) when K > 0 ->
	case dict:find({Begin, End},Cache) of
		{ok, Ps} ->  
			Pid ! {?reply, get_k_shortest_path, Ps},
			{noreply, S};
		error -> Ps = astar:yen(G, Begin, End,K),
			NewCache = dict:store({Begin, End}, Ps, Cache),
			Pid ! {?reply, get_k_shortest_path, Ps},
			{noreply, S#proxyState{cache=NewCache}}
	end;
handle_info({vertex, V, Pid}, S=#proxyState{graph=G}) ->
	Pid ! {?reply, vertex, digraph:vertex(G, V)},
	{noreply, S};
handle_info({subscribe, get_info, Time, once, Receiver}, S) ->
	timer:send_after(Time, {get_info, Receiver}),
	{noreply, S};
handle_info({subscribe, get_info, Interval, repeat, Receiver}, S) ->
	timer:send_interval(Interval, {get_info, Receiver}),
	{noreply, S};
handle_info({get_info, Pid},  S) ->
	Pid ! {info, ?ID, S},
	{noreply, S};
% default callback for messages.
handle_info(_Message, S) ->
    {noreply, S}.

% default code_change callback.
code_change(_OldVsn, State, _Extra) ->
    {ok, State}.

% default terminate callbacks
terminate(normal, _S) ->
    io:format("Modum client stopped normally~n");
terminate(shutdown, _S) ->
    io:format("Modum client got shutdown~n");
terminate(Reason, _S) ->
    io:format("Modum client got killed with reason:~n"),
	io:write(Reason).


%%%%%%%%%%%%%%%%%%%%%%
% Internal Functions %
%%%%%%%%%%%%%%%%%%%%%%

createGraph({NodeStates, LinkStates}) ->
	G = digraph:new(),
	N = addNodeToGraph(G, [], NodeStates),
	L = addLinkToGraph(G, [], LinkStates),
	{ok, {G,N,L}}.

addNodeToGraph(G, NIds,[N | NodeStates]) ->
	digraph:add_vertex(G, NId=N#nodeState.id, N),
	addNodeToGraph(G,[NId|NIds],NodeStates);
addNodeToGraph(_G,N,[]) ->
	N.

addLinkToGraph(G, LIds,[#linkState{id=Id} | LinkStates]) ->
	{?reply, state,  L = #linkState{connection=#connection{from=From,to=To}}} = gen_server:call(Id, state, ?callTimeout),
	case (From == undefined) or (To == undefined) of
		true -> 
			addLinkToGraph(G, LIds,LinkStates);
		false -> 
			digraph:add_vertex(G,Id,L),
			digraph:add_edge(G, {L#linkState.id,1},From, Id,L#linkState.length / 2.0),
			digraph:add_edge(G, {L#linkState.id,2},Id, To, L#linkState.length / 2.0),
			addLinkToGraph(G, [Id|LIds],LinkStates)
	end;
addLinkToGraph(_G,L,[]) ->
	L.

updateLinkStates(Dict, []) ->
	Dict;
updateLinkStates(Dict, [LinkInfo=#linkInformationType{id=Id, density=Density} | Rest]) ->
    list_to_float(Density) == 0.0 orelse io:format("new density for link ~w: ~w~n",[list_to_atom(Id),list_to_float(Density)]),
	NewDict = dict:store(list_to_atom(Id), LinkInfo, Dict),
	% inform link immediately:
	gen_server:cast(list_to_atom(Id), traffic_update),
	updateLinkStates(NewDict,Rest);
updateLinkStates(_, _) ->
	{error, unknown_format}.
