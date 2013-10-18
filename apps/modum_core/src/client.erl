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
-module(client).
-behaviour(gen_server).

-export([init/1, handle_call/3, handle_cast/2,
         handle_info/2, code_change/3, terminate/2]).
-export([start_link/1, stop/1]).
-include("states.hrl").

-define(port, 4567).
-define(ip, {127,0,0,1}).
-define(timeout, 6000).
-define(max_sessions, 1000).

stop(Id) -> 
	gen_server:call(Id, stop, ?callTimeout).

start_link(ComState=#comState{init=Init,ip=Ip,port=Port,parser=Parser})
	when is_function(Init) and
		 is_tuple(Ip) and
		 is_integer(Port) and
		 is_function(Parser) ->
	gen_server:start_link(?MODULE, [ComState], []);
start_link(Other) ->
	io:format("Invalid com state for client: ~w~n", [Other]).
	
init([ClientState=#comState{init=Init}]) ->
    %% To know when the parent shuts down
    process_flag(trap_exit, true),
    case Init(ClientState) of
		ok -> 
			io:format("Initialized client for ~w:~w at ~w~n", [ClientState#comState.ip, ClientState#comState.port, self()]),
			{ok, ClientState};
		error ->
			io:format("Initialization of client ~w failed!", [ClientState]),
			{stop, normal, ok, ClientState}
	end.

handle_call({transmit, Payload}, _From, S=#comState{parser=Parser}) ->
	Result = Parser(Payload, S),
	{reply, {transmit, Result}, S};
handle_call(stop, _From, S) ->
    {stop, normal, ok, S};
handle_call(_Message, _From, S) ->
    {noreply, S}.

handle_cast({transmit, Payload}, S=#comState{parser=Parser}) ->
	Parser(Payload, S),
	{noreply, S};
handle_cast(_Message, S) ->
    {noreply, S}.
	
handle_info(Message, S) ->
	io:format("Client received unknown message ~w~n",[Message]),
	{noreply, S}.

code_change(_OldVsn, State, _Extra) ->
    {ok, State}.

terminate(normal, S) ->
    io:format("Client ~w terminated normally~n",[S]);
terminate(shutdown, S) ->
    io:format("Client ~w got shutdown~n",[S]);
terminate(Reason, S) ->
    io:format("Client ~w got killed with reason: ~w~n", [S,Reason]).