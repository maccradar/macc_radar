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
-module(core_test).

-export([current_flow_test/0,test_xmlrpc/5,linkState/1, digraph/0, create_vehicles/4, testShortestPath/1,get_travel_times/1,simulate_traffic/1,test_avg/4, test_yen/1, test_yaws/0, test_detergent_server/0, test_soap/4, test_soap_client/0]).

-include("states.hrl").
-include_lib("eunit/include/eunit.hrl").
-include_lib("blackboard/include/blackboard_records.hrl").
-include_lib("util/include/debug.hrl").
-include_lib("yaws/include/yaws.hrl").

linkState(Link) ->
	Link ! {state,self()},
	receive
		{reply,state,State} ->
			State;
		Other ->
			Other
	after 5000 ->
		nothing_received
	end.

test_avg(M, F, A, N) when N > 0 ->
    L = test_loop(M, F, A, N, []),
    Length = length(L),
    Min = lists:min(L),
    Max = lists:max(L),
    Med = lists:nth(round((Length / 2)), lists:sort(L)),
    Avg = round(lists:foldl(fun(X, Sum) -> X + Sum end, 0, L) / Length),
    io:format("Range: ~b - ~b mics~n"
	      "Median: ~b mics~n"
	      "Average: ~b mics~n",
	      [Min, Max, Med, Avg]),
    Med.

test_loop(_M, _F, _A, 0, List) ->
    List;
test_loop(M, F, A, N, List) ->
    {T, _Result} = timer:tc(M, F, A),
    test_loop(M, F, A, N - 1, [T|List]).

digraph() ->
	modum_proxy ! {get_shortest_path, ?node_in, ?node_out, self()},
	receive
		{?reply, get_shortest_path, P} -> 
			io:format("~w~n", [P])
	end.

simulate_traffic(From, To) ->
	create_vehicles(500, {0,10}, {From, To}, auto),
	create_vehicles(500, {10,3600}, {From, To}, auto),
	create_vehicles(500, {3600,7200}, {From, To}, auto).
	
simulate_traffic(scenario0)->
	create_vehicles(1000,{0,10}, {?node_in,?node_out},auto),
	create_vehicles(500,{10,3600}, {?node_in,?node_out},auto),
	create_vehicles(3000,{3600,7200}, {?node_in,?node_out},auto).

create_vehicles(0,{_,_},_, _Mode)->
	ok;
create_vehicles(NrVehicles,Time = {departure_time, {From,To}}, {O,D,K}, Mode) ->
	DepartureTime = From+random:uniform(To-From),
	VehicleState = #vehicleState{departureTime=DepartureTime,origin=#location{resource=O,position=?link_in},destination=#location{resource=D,position=?link_out},location=#location{resource=O,position=?link_in},id=list_to_atom("vehicle"++ integer_to_list(From)++integer_to_list(NrVehicles)), length=5,maxSpeed=100},
	modum_main:create_vehicle({VehicleState,Mode,K}),
	create_vehicles(NrVehicles-1,Time,{O,D,K}, Mode);
create_vehicles(NrVehicles, Time = {arrival_time, ArrivalTime}, {O, D,K}, Mode) ->
	VehicleState = #vehicleState{arrivalTime = ArrivalTime, departureTime=?undefined,origin=#location{resource=O,position=?link_in},destination=#location{resource=D,position=?link_out},location=#location{resource=O,position=?link_in},id=list_to_atom("vehicle"++ integer_to_list(NrVehicles)), length=5,maxSpeed=100},
	modum_main:create_vehicle({VehicleState,Mode,K}),
	create_vehicles(NrVehicles-1, Time,{O,D,K}, Mode).
	
get_travel_times(Link)->
	Link ! {being,self()},
	receive 
		{?reply, being, LB} -> link_model:get_travel_times(LB);
		_ -> ?undefined
	end.

dlog_init_test() ->
	?CREATE_DEBUG_TABLE.

dlog_test() ->
	?DLOG("test").

	
current_flow_test() ->
	BB = bb_ets:create("BB_test_current_flow"),
	PhId1 = pheromone:create([BB], 10000, #info{data={linkx_flow,cumul},tags=[flow]}).
evaporation_test() ->
 VehicleId = vehicle1,
 Time1 = 1000,
 Time2 = 2000,
 BB = bb_trafficflow:create("BB_testEvaporation"),
 PhId1 = pheromone:create([BB], 5000,#vehicle_info{vehicle=VehicleId, arrival_time=Time1,data=void}),
 PhId2 = pheromone:create([BB], 5000,#vehicle_info{vehicle=VehicleId, arrival_time=Time2,data=void}),
 receive after 1000 -> slept end,
 ?assert(is_process_alive(PhId1)==false),
 ?assert(is_process_alive(PhId2)==true).

testShortestPath(Length) ->
	G = digraph:new(),
	Start = digraph:add_vertex(G),
	End = populateGraph(G,Start, Length),
	{Time,_V} = timer:tc(digraph, get_short_path, [G,Start,End]),
	io:format("Shortest path took ~w microseconds.~n", [Time]).
	
populateGraph(_G,End, 0) ->
	End;
populateGraph(G,Start, Length) ->
	V1 = digraph:add_vertex(G),
	digraph:add_edge(G,Start, V1),
	populateGraph(G,V1,Length-1).

test_yen(K) ->
	G = digraph:new(),
	digraph:add_vertex(G,c,#nodeState{shape=[#point{x=0,y=0}]}),
	digraph:add_vertex(G,d,#nodeState{shape=[#point{x=3,y=0}]}),
	digraph:add_vertex(G,e,#nodeState{shape=[#point{x=3,y=-1}]}),
	digraph:add_vertex(G,f,#nodeState{shape=[#point{x=7,y=0}]}),
	digraph:add_vertex(G,g,#nodeState{shape=[#point{x=6,y=-1}]}),
	digraph:add_vertex(G,h,#nodeState{shape=[#point{x=8,y=-1}]}),
	digraph:add_edge(G,c,d,3),
	digraph:add_edge(G,c,e,2),
	digraph:add_edge(G,d,f,4),
	digraph:add_edge(G,e,d,1),
	digraph:add_edge(G,e,f,2),
	digraph:add_edge(G,e,g,3),
	digraph:add_edge(G,f,g,2),
	digraph:add_edge(G,f,h,1),
	digraph:add_edge(G,g,h,2),
	astar:yen(G,c,h,K).
	
test_yaws() ->
	application:set_env(yaws, embedded, true),
	application:set_env(yaws, id, "Yaws MODUM Server"),
	yaws:start_embedded("/research/workspace/Erlang/apps/yaws/www").

test_xmlrpc(Sv, Ip, Port, Xsd, XmlFile) ->
	{ok, Model} = erlsom:compile_xsd_file(Xsd),
	{ok, Client} = supervisor:start_child(Sv, [#comState{init=inits:dummy_client(), ip=Ip, port=Port, parser=parsers:xmlrpc_client()}]),
	{transmit, Response} = gen_server:call(Client,{transmit, {Model, XmlFile}}),
	Response.
	
test_soap(_Header,
        _A,
        _Action, 
        _SessionValue) ->
    {ok, undefined, "ok"}.

test_soap_client() ->
inets:start(),
Data = "<?xml version=\"1.0\" ?>
		 <S:Envelope xmlns:S=\"http://schemas.xmlsoap.org/soap/envelope/\">
		  <S:Body>
		  <ns2:provideMapInfoRequest xmlns:ns2=\"http://mobilephone.modum.mm/\">
		   <mapInfoRequest>
		    <deviceID>884455</deviceID>
			<requestID>22000</requestID>
			<timestamp>2013-08-02T16:06:17.182+02:00</timestamp>
			<originID>1234</originID>
			<destinationID>6789</destinationID>
			<preferences>
			 <departureTime>2013-08-02T16:06:27.182+02:00</departureTime>
			 <arrivalTime>2013-08-02T16:06:37.182+02:00</arrivalTime>
			 <preferredTransport>PEDESTRIAN</preferredTransport>
			</preferences>
		   </mapInfoRequest>
		  </ns2:provideMapInfoRequest>
		  </S:Body>
		 </S:Envelope>",
{ok, Result} = httpc:request( post, {"http://localhost:9000/map_info_request.yaws", [{"Host" , "http://localhost:9000"}, {"SOAPAction", "\"provideMapInfoRequest\""}], "text/xml", Data}, [ {timeout, 5000} ], [{body_format, binary}]),
io:format("Soap Response is ~p~n", [Result]),
Result.	
% test_detergent_client() ->
	% detergent:call("MobilePhoneResponseInterfaceService.wsdl", "provideMapInfoResponse",[#'P:mapInfoResponse'{deviceID="bla", requestID=10000,timestamp="1000", summary=#'P:summary'{co2=100,trafficJam=100}}],#call_opts{prefix="P"}).
	
test_detergent_server() ->
	detergent_server:start_link([{{parsers,soap_server_request},"/research/workspace/Erlang/www/MobilePhoneRequestInterfaceService.wsdl"}]).