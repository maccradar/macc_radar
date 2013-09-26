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
%% Author: jphilips
%% Created: Oct 5, 2012
%% Description: TODO: Add description to modum_main
-module(modum_main).
-behaviour(application).

%% Application callbacks
-export([start/2, stop/1]).
-export([loop/1, create_vehicle/1, start_concurix/0]).
-include("states.hrl").
-include("xml.hrl").
-include_lib("util/include/debug.hrl").

-define(id, modum).

%% ===================================================================
%% Application callbacks
%% ===================================================================

start(normal, _Args) ->
	?CREATE_DEBUG_TABLE,
	Main = spawn(?MODULE,loop,[{?undefined,?undefined,?undefined}]),
	register(?id, Main),
	{ok, ProjectDir} = application:get_env(modum_core, project_dir),
	{ok, ComDir} = application:get_env(modum_core, com_dir),
	{ok, Xsd} = application:get_env(modum_core, xsd),
	{ok, Map} = application:get_env(modum_core, map),
	{ok, Server} = application:get_env(modum_core, server),
	{ok, Ip} = application:get_env(modum_core, ip),
	{ok, ForecastPort} = application:get_env(modum_core, forecast_port),
	{ok, OptimalPathPort} = application:get_env(modum_core, optimal_path_port),
	{ok, UserRequestPort} = application:get_env(modum_core, user_request_port),
	{ok, UserResponsePort} = application:get_env(modum_core, user_response_port),
	{ok, UserResponseIp} = application:get_env(modum_core, user_response_ip),
	{ok, TrafficUpdatePort} = application:get_env(modum_core, traffic_update_port),
	{ok, TrafficUpdateIp} = application:get_env(modum_core, traffic_update_ip),
	{ok, WwwPort} = application:get_env(modum_core, www_port),
	
	% start supervisor for all clients
	{ok, ClientSv} = client_sv:start_link(simple),
	% start user response client (detergent)
	{ok, UserResponseClient} = supervisor:start_child(ClientSv, [#comState{init=inits:user_response_client(), ip=UserResponseIp, port=UserResponsePort, parser=parsers:user_response_client()}]),
	io:format("User response client: ~w~n", [UserResponseClient]),
	{ok, TrafficUpdateClient} = supervisor:start_child(ClientSv, [#comState{init=inits:traffic_update_client(), ip=TrafficUpdateIp, port=TrafficUpdatePort, parser=parsers:traffic_update_client()}]),
	% start supervisor for all servers
	{ok, ServerSv} = server_sv:start_link(simple),
	% do we need a simulation server or do we connect to sumo?
	TrafficUpdateServer = case Server of 
		real -> ?undefined;
		_ -> supervisor:start_child(ServerSv, [#comState{init=inits:xmlrpc_server(),ip=TrafficUpdateIp,port=TrafficUpdatePort,parser=parsers:traffic_update_server(filename:join([ProjectDir,ComDir,Xsd]))}])
	end,	
	% start web server (yaws)
	{ok, WebServer} = supervisor:start_child(ServerSv, [#comState{init=inits:web_server(), ip=Ip, port=WwwPort, parser=parsers:web_server()}]),
	% start forecast server for UNIMAN
	{ok, ForecastServer} = supervisor:start_child(ServerSv,[#comState{init=inits:xmlrpc_server(),ip=Ip, port=ForecastPort, parser=parsers:forecast_server(filename:join([ProjectDir,ComDir,Xsd]))}]),
	% start user request server for UNIMAN
	{ok, OptimalPathServer} = supervisor:start_child(ServerSv,[#comState{init=inits:xmlrpc_server(),ip=Ip, port=OptimalPathPort, parser=parsers:optimal_path_server(filename:join([ProjectDir,ComDir,Xsd]))}]),
	% start user request server (yaws)
	{ok, UserRequestServer} = supervisor:start_child(ServerSv,[#comState{init=inits:user_request_server(),ip=Ip, port=UserRequestPort, parser=parsers:user_request_server()}]),
	io:format("User request server: ~w~n", [UserRequestServer]),
	% io:format("Response from yaws: ~w~n",[Result]),
	?id ! {init, 
		   self(), 
		   filename:join([ProjectDir,ComDir,Xsd]), 
		   filename:join([ProjectDir,ComDir,Map]), 
		   #proxyState{clients=[{?user_response_client,UserResponseClient},
								{?traffic_update_client,TrafficUpdateClient}],
					   servers=[{?user_request_server,UserRequestServer},
								{?web_server,WebServer},
								{?forecast_server,ForecastServer},
								{?optimal_path_server,OptimalPathServer},
								{?traffic_update_server,TrafficUpdateServer}]}
		  },
	receive
		{?reply, init, Result} ->
			{Result, Main}
	after 10000 -> error
	end.
	
stop(_State) ->
    ok.

start_concurix() ->
	concurix_runtime:start().
	
create_vehicle({VehicleState,Mode,K}) ->
	?id ! {get_vehicle_sv, self()},
	receive
		{?reply, get_vehicle_sv, VSV} ->
			VSV
	end,
	{ok, _Child} = supervisor:start_child(VSV, [VehicleState,Mode,K]).
	
% internal loop of main process
% - send {init, {Xsd, Map}} to initialise the program
% - send stop to stop the program. This will kill/shutdown all its child processes 
loop(Data={NSV,LSV,VSV}) ->
	receive 
		{init,Pid, Xsd, Map, ProxyState} ->
			{ok, ProjectDir} = application:get_env(modum_core,project_dir),
			{ok, IncludeDir} = application:get_env(modum_core,include_dir),
			erlsom:write_xsd_hrl_file(Xsd,filename:join([ProjectDir,IncludeDir,"xml.hrl"])),
			case erlsom:compile_xsd_file(Xsd) of
			{ok, Model} ->
				case init(Map, ProxyState#proxyState{model=Model}) of
					{ok, NewNSV, NewLSV, NewVSV} ->
						io:format("Initialization finished, entering main loop.~n"),
						Pid ! {?reply, init, ok},
						loop({NewNSV, NewLSV, NewVSV});
					error ->
						io:format("Initialization failed!~n"),
						Pid ! {?reply, init, error};
					Other ->
						io:format("Received unknown format ~w~n", [Other]),
						Pid ! {?reply, init, Other}
				end;
			{error, Error} ->
				Pid ! {?reply, init,Error}
			end;
		{get_node_sv, Pid} ->
			Pid ! {?reply, get_node_sv, NSV},
			loop(Data);
		{get_link_sv, Pid} ->
			Pid ! {?reply, get_link_sv, LSV},
			loop(Data);
		{get_vehicle_sv, Pid} ->
			Pid ! {?reply, get_vehicle_sv, VSV},
			loop(Data);		
		stop ->
			die;
		M ->
			io:format("Received unknown message ~p~n", [M]),
			loop(Data)
	end.

init(Map, ProxyState=#proxyState{model=Model}) ->
	case erlsom:scan_file(Map, Model) of
		{ok, MapData, _} ->
			{Nodes, Links} = parse_map(MapData),
			modum_proxy:start_link(ProxyState), % need supervisor?
			{ok, NSV} = node_holon_sv:start_link(simple),
			lists:foreach(fun(Node)->supervisor:start_child(NSV, [Node]) end, Nodes),
			{ok, LSV} = link_holon_sv:start_link(simple),
			lists:foreach(fun(Link)->supervisor:start_child(LSV, [Link]) end, Links),
			{ok, VSV} = vehicle_holon_sv:start_link(simple),
			U = atom_to_list(?undefined),
			FilterN = [F || F <- Nodes, F#nodeState.coordinates /= [{U,U}]],
			io:format("Filtered ~w nodes with undefined coordinates~n", [length(Nodes)-length(FilterN)]),
			modum_proxy:create_graph({FilterN, Links}),
			add_turning_fractions({FilterN, Links}),
			{ok, NSV, LSV, VSV};
		{error, Error} ->
			Error
	end.

parse_map(MapData) ->
	Links = MapData#topologyMap.link,
	Nodes = MapData#topologyMap.node,
	{NodeStates, _,_,Dict} = lists:foldl(
		fun(#nodeType{id=Id, nodeDesc=Desc, linkPair=LP, coordinates=Coords}, {NodeDict, NodeIn, NodeOut, LinkConnections}) -> 
			LF =  lists:foldl(
				fun(#linkPairType{idfrom=From, idto=To}, {ConnectionsList,LinkConnections2,Id1}) -> 
					Id2 = Id1, % get_id(list_to_atom(From), list_to_atom(To), Id1),
					LC=dict:store({list_to_atom(From),out},Id2,LinkConnections2),
					LC2=dict:store({list_to_atom(To),in},Id2,LC),
			
			{[#connection{from=list_to_atom(From), to=list_to_atom(To)} | ConnectionsList],LC2,Id2}
					end,
					{[],LinkConnections, list_to_atom(Id)}, LP),
			  NewCoords = [{list_to_float(Lat),list_to_float(Lon)} || #coordinatesType{lat=Lat, lon=Lon} <- Coords],
			  NewNode = #nodeState{id=element(3,LF), desc=Desc, connections=element(1,LF)++get_connections(element(3,LF),NodeIn,NodeOut), coordinates=NewCoords},
			  NewNodeDict = dict:store(element(3,LF),NewNode,NodeDict),
			  {NewNodeDict, update_node(?node_in,NewNode,NodeIn), update_node(?node_out,NewNode,NodeOut),element(2,LF)}
		end, {dict:new(), ?undefined, ?undefined, dict:new()}, Nodes),	
	{LinkStates, FinalNodeDict} = lists:foldl(
		fun(#linkType{id=Id, numLanes=Lanes, length=Length, maxSpeed=MaxSpeed, shape=ShapeString, linkDesc=Desc, roadType=RoadType, coordinates=Coords}, {LinkList, NodeDict}) ->
			Shape = shape_to_points(ShapeString),
			[FirstS | _] = Shape,
			[LastS | _] = lists:reverse(Shape),
			LinkId2 = list_to_atom(Id),
			FromNode = dict:fetch({LinkId2,in}, Dict),
			FromNS = dict:fetch(FromNode, NodeDict),
			ToNode = dict:fetch({LinkId2,out}, Dict),
			ToNS = dict:fetch(ToNode, NodeDict),
			NewNodeDict1 = dict:store(FromNode,FromNS#nodeState{shape=[FirstS, FirstS]},NodeDict),
			NewNodeDict2 = dict:store(ToNode, ToNS#nodeState{shape=[LastS, LastS]}, NewNodeDict1),
			NewCoords = [{Lat,Lon} || #coordinatesType{lat=Lat, lon=Lon} <- Coords],
			{[#linkState{id=LinkId2, desc=Desc, numLanes=Lanes, length=list_to_float(Length), maxAllowedSpeed=list_to_float(MaxSpeed),connection=#connection{from=FromNode, to=ToNode}, shape=Shape, roadType=RoadType, coordinates=NewCoords} | LinkList], NewNodeDict2}
		end, {[], NodeStates}, Links),
	io:format("Parsed ~B nodes and ~B links~n",[length(Nodes), length(LinkStates)]),
	{[X || {_,X} <- dict:to_list(FinalNodeDict)],LinkStates}.
		
update_node(Id, NewNode=#nodeState{id=Id},_) ->
	NewNode;
update_node(_, _,OldNode) ->
	OldNode.
	
get_connections(?node_in, #nodeState{connections=C}, _) -> 
	C;
get_connections(?node_out, _ , #nodeState{connections=C}) -> 
	C;
get_connections(_, _ , _) -> 
	[].
	
get_id(?link_in,_To,_Id)->
	?node_in;
get_id(_In,?link_out,_Id)->
	?node_out;
get_id(_In,_Out, Id) ->
	Id.
	
shape_to_points(Shape) ->
	Ps = string:tokens(Shape," "),
	F = fun(P) -> [X,Y] = string:tokens(P, ","), #point{x=list_to_float(X),y=list_to_float(Y)} end,
	lists:map(F, Ps).

add_turning_fractions({Nodes, Links}) ->
	{ok, ProjectDir} = application:get_env(modum_core,project_dir),
	{ok, ComDir} = application:get_env(modum_core,com_dir),
	{ok, TurnDefsFile} = application:get_env(modum_core,turn_defs),
	{ok, Xml} = file:read_file(filename:join([ProjectDir,ComDir,TurnDefsFile])),
	{ok, {_Tag, _Att, Content}, _Tail} = erlsom:simple_form(Xml),
	FromTos = [{list_to_atom(FromId), whereis(list_to_atom(FromId)),[{list_to_atom(ToId), list_to_float(Prob)/100.0} || {"toEdge",[{"probability", Prob},{"id", ToId}],[]} <- Tos]} || {"fromEdge",[{"id",FromId}],Tos} <-Content],
	NodeFun = fun(#nodeState{id=NodeId}) ->
		case gen_server:call(NodeId, connections) of
		{?reply, connections, [#connection{from=From, to=ToId}]} -> 
				NodeId ! {add_turning_fractions, From, [{ToId,1.0}]};
		{?reply, connections, Connections} -> UTurnFun =
				fun	(#connection{from=U1,to=U2})-> 
						([C || C <- atom_to_list(U1), C =/= $-] == [C || C <- atom_to_list(U2), C =/= $-]) andalso 
						(NodeId ! {add_turning_fractions, U1, [{U2,0.0}]});
					(_) -> use_turndefs 
				end,
				lists:foreach(UTurnFun, Connections)
		end
	end,
	lists:foreach(NodeFun, Nodes),
	TurnFun = fun({FromId2,Pid,Tos}) when Pid =/= undefined -> 
		Pid ! {downstream_connections, self()},
		NodeId = receive
			{?reply, downstream_connections, [Node]} -> Node
			end,
		NodeId ! {add_turning_fractions, FromId2, Tos};
			({_,_,_}) ->
				ignore_undefined_links
		end,
	lists:foreach(TurnFun,FromTos).
