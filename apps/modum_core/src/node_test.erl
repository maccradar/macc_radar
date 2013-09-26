-module(node_test).
-include_lib("eunit/include/eunit.hrl").

-compile(export_all).

setup() ->
	file:set_cwd("/research/workspace/Erlang/macc_radar/"),
	io:format("Current working dir: ~p~n", [file:get_cwd()]),
	application:start(util),
	application:start(blackboard),
	application:start(modum_core),
	ok.
	
consistency_test() ->
	setup(),
	?assertMatch({reply,{check_consistency,nodes},true}, gen_server:call(modum_proxy, {check_consistency,nodes})).
	
% distance_test() ->
	% setup(),
	% {Links, Nodes} = modum_proxy:get_status_info(),
	% ?assertEqual(