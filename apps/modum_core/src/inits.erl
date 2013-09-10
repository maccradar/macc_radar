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
-module(inits).

-include("states.hrl").

-export([dummy_client/0, user_request_server/0, web_server/0, user_response_client/0, traffic_update_client/0, xmlrpc_server/0]).

-define(timeout, 6000).
-define(max_sessions, 1000).

dummy_client() ->
	fun(#comState{}) ->
		ok
	end.

user_response_client() ->
	fun(#comState{}) ->
		ok
	end.

xmlrpc_server() ->
	fun(State=#comState{ip=Ip,port=Port}) ->
		case xmlrpc:start_link(Ip, Port, ?max_sessions, ?timeout, {parsers,xmlrpc_callback}, State) of
			{ok, _Pid} -> ok;
			_ -> error
		end
	end.

traffic_update_client() ->
	fun(#comState{}) ->
		ok
	end.
	
user_request_server() ->
	{ok, WwwDir} = application:get_env(modum_core, www_dir),
	{ok, RequestWsdl} = application:get_env(modum_core, request_wsdl),
	fun(#comState{}) ->
		case yaws_soap_srv:setup({parsers, soap_callback}, filename:join([WwwDir,RequestWsdl]),"P") of
			ok -> ok;
			_ -> error
		end
	end.

web_server() ->
	fun(ComState=#comState{ip=Ip, port=Port}) ->
		{ok, ProjectDir} = application:get_env(modum_core, project_dir),
		{ok, EbinDir} = application:get_env(modum_core, ebin_dir),
		{ok, WwwDir} = application:get_env(modum_core, www_dir),
		{ok, IncludeDir} = application:get_env(modum_core, include_dir),
		Root = filename:join([ProjectDir,WwwDir]),
		Id = "embedded",
		GconfList = [{enable_soap,true},
					 {id, Id},
					 {trace,false},
					 {tmpdir, Root}, 
					 {logdir, Root},
					 {flags, [{tty_trace, false}, {copy_errlog, true}]}
					],
		SconfList = [{port, Port},
					 {servername, "localhost"},
					 {dir_listings, true},
					 {listen, Ip},
					 {flags,[{auth_log,false},{access_log,false}]},
					 {docroot, Root},
					 {appmods, [{"cfs", yaws_sse_cfs}, {"history", yaws_sse_history}, {"density", yaws_sse_density}, {"intentions", yaws_sse_intentions}]},				 
					 {ebin_dir, filename:join([ProjectDir,EbinDir])}
					],
		{ok, SCList, GC, ChildSpecs} =
			yaws_api:embedded_start_conf(Root, SconfList, GconfList, Id),
		{ok, Sup} = server_sv:start_link({one_for_all, 0, 1}), % child supervisor to manage yaws processes
		L  = [supervisor:start_child(Sup, Ch) || Ch <- ChildSpecs],
		io:format("Started Yaws processes: ~w~n",[L]),
		yaws_api:setconf(GC, SCList),
		ok
	end.