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
-module(parsers).

-include("xml.hrl").
-include("states.hrl").
-include("soap_request_response.hrl").
-include_lib("detergent/include/detergent.hrl").
-include_lib("erlsom/src/erlsom_parse.hrl").

-export([dummy_server/0, traffic_update_client/0, traffic_update_server/1, forecast_server/1, optimal_path_server/1, user_request_server/0, soap_callback/4, xmlrpc_callback/2, user_response_client/0, web_server/0, xmlrpc_client/0]).

dummy_server() ->
	fun(_Request) ->
		ok
	end.

web_server() ->
	fun(_Request) ->
		ok
	end.

xmlrpc_client() ->
	fun({Model, XmlFile}, #comState{ip=Ip,port=Port}) ->
		io:format("Loading xml ~p~n",[XmlFile]),
		{ok, Request, _} = erlsom:scan_file(XmlFile, Model),
		{ok, Command} = erlsom:write(Request, Model),
		EncCommand = base64:encode_to_string(Command),
		{ok, Socket} = gen_tcp:connect(Ip,Port,[{active, false}]),
		case xmlrpc:call(Socket, "/", {call, forecastrequest, [{base64,EncCommand}]}) of
			{ok,{response,[EncResponse]}} ->
				Response = base64:decode_to_string(EncResponse),
				{ok, Result, _} = erlsom:scan(Response, Model),
				io:format("Result: ~p~n", [Result]),
				Result;
			Error -> 
				io:format("Error receiving response ~w~n", [Error]),
				?undefined
		end;
		(Payload,_) ->
			io:format("Unknown payload: ~w~n", [Payload]),
			?undefined
	end.	
	
traffic_update_client() ->
	{ok, ProjectDir} = application:get_env(modum_core,project_dir),
	{ok, ComDir} = application:get_env(modum_core,com_dir),	
	fun({async, Model},#comState{ip=Ip,port=Port}) when is_record(Model,model) ->
		{ok, Request, _} = erlsom:scan_file(filename:join([ProjectDir, ComDir, "modum_updateRequest.xml"]), Model),
		{ok, Command} = erlsom:write(Request, Model),
		EncCommand = base64:encode_to_string("<?xml version=\"1.0\" encoding=\"utf-8\" ?>"++Command),
		case gen_tcp:connect(Ip,Port,[{active, false}]) of
			{ok, Socket} -> 
				io:format("Request: ~p~n", [Request]),
				case xmlrpc:call(Socket, "/RPC2", {call, request, [{base64, EncCommand}]}, false, 300000) of
					{ok,{response,[EncResponse]}} ->
						Response = base64:decode_to_string(EncResponse),
						{ok, Result, _} = erlsom:scan(Response, Model),
						gen_server:cast(modum_proxy, {traffic_update_response, Result});
					Error -> 
						io:format("Error receiving response ~p~n", [Error]),
						gen_server:cast(modum_proxy, {traffic_update_response, ?undefined})
			end;
			{error, Error} ->
				io:format("Could not open socket ~w:~w due to: ~w~n", [Ip,Port,Error]),
				gen_server:cast(modum_proxy, {traffic_update_response, ?undefined})
		end;
		({sync,Model},#comState{ip=Ip,port=Port}) when is_record(Model,model) ->
		{ok, Request, _} = erlsom:scan_file(filename:join([ProjectDir, ComDir, "modum_updateRequest.xml"]), Model),
		{ok, Command} = erlsom:write(Request, Model),
		EncCommand = base64:encode_to_string(Command),
		{ok, Socket} = gen_tcp:connect(Ip,Port,[{active, false}]),
		io:format("command: ~p~n", [Command]),
		case xmlrpc:call(Socket, "/", {call, request, [{base64, EncCommand}]}) of
			{ok,{response,[EncResponse]}} ->
				Response = base64:decode_to_string(EncResponse),
				{ok, Result, _} = erlsom:scan(Response, Model),
				Result;
			Error -> 
				io:format("Error receiving response ~p~n", [Error]),
				?undefined
		end;
		(Payload,_) ->
			io:format("Unknown payload: ~w~n", [Payload]),
			?undefined
	end.	
	
traffic_update_server(Xsd) ->
	fun(Request) ->
		{ok, ProjectDir} = application:get_env(modum_core, project_dir),
		{ok, ComDir} = application:get_env(modum_core,com_dir),
		case erlsom:compile_xsd_file(Xsd) of
			{ok, Model} ->
				case erlsom:scan(Request, Model) of
					{ok, Xml, _} ->
						{ok, Response, _} = erlsom:scan_file(filename:join([ProjectDir,ComDir,"modum_updateResponse.xml"]), Model),
						C = Response#updateResponse{extrastatus=Xml#updateRequest.commandString},
						case erlsom:write(C, Model) of
							{ok, Command} ->
								Command;
							{error, Error} ->
								Error
						end;
					{error, Error} ->
						Error
				end;
			{error, Error} ->
				Error
		end
	end.

forecast_server(Xsd) ->
	fun(Request) ->
		case erlsom:compile_xsd_file(Xsd) of
			{ok, Model} ->
				case erlsom:scan(Request, Model) of
					{ok, Xml, _} ->
						TimeWindow = Xml#forecastRequest.timeWindow,
						F = modum_proxy:get_forecast(TimeWindow),
						Forecast = prepare_forecast_output(F),
						C = #forecastResponse{link=Forecast},
						case erlsom:write(C, Model) of
							{ok, Command} ->
								Command;
							{error, Error} ->
								Error
						end;
					{error, Error} ->
						Error
				end;
			{error, Error} ->
				Error
		end
	end.
	
prepare_forecast_output(Forecast) ->
	[#linkForecastType{id=Id,interval=[#forecastIntervalType{timeStart=T, timeStop=T+TimeStep, travelTime=TravelTime} || {T,TravelTime} <- TravelTimes]} || {Id, TravelTimes, TimeStep} <- Forecast].

optimal_path_server(Xsd) ->
	fun(Request) ->
		case erlsom:compile_xsd_file(Xsd) of
			{ok, Model} ->
				case erlsom:scan(Request, Model) of
					{ok, Xml, _} ->
						C = #userResponse{userId="ok"},
						case erlsom:write(C, Model) of
							{ok, Command} ->
								Command;
							{error, Error} ->
								Error
						end;
					{error, Error} ->
						Error
				end;
			{error, Error} ->
				Error
		end
	end.

user_response_client() ->
	{ok, ResponseWsdl} = application:get_env(modum_core, response_wsdl),
	fun(_Data,#comState{}) ->
		% eerst inpakken
		detergent:call(ResponseWsdl, "provideMapInfoResponse",[#'P:mapInfoResponse'{deviceID="bla", requestID=10000,timestamp="1000", summary=#'P:summary'{co2=100,trafficJam=100}}],#call_opts{prefix="P"})
	end.

user_request_server() ->
	fun(A) ->
		io:format("Received: ~p~n", [A]),
		yaws_rpc:handler_session(A, {?MODULE, soap_callback})
	end.

	
% parser callbacks
soap_callback(Header, Body, Action, SessionValue) ->
	io:format("SOAP server received: ~w,~w,~w,~w~n", [Header,Body,Action,SessionValue]),
	
	% parse SOAP request
	% Message = parse_soap()
	% modum_proxy ! Message
	% vehicle aanmaken
	% oplossing zoeken
	
	{ok, undefined,#'P:provideMapInfoRequestResponse'{} }.
	
% Handle RPC call
xmlrpc_callback(#comState{parser=Parser}, {call, request, [{base64,Command}]}) ->
	Request = base64:decode_to_string(Command),
	io:format("Client request: ~p~n", [Request]),
	Response = Parser(Request),
	io:format("Server response: ~p~n", [Response]),
	EncCommand = base64:encode_to_string(Response),
    {false, {response, [EncCommand]}};

% Assume base64	
xmlrpc_callback(#comState{parser=Parser}, {call, request, [Command]}) ->
	Request = base64:decode_to_string(Command),
	io:format("Client request 2: ~p~n", [Request]),
	Response = Parser(Request),
	io:format("Server response 2: ~p~n", [Response]),
	EncCommand = base64:encode_to_string(Response),
    {false, {response, [EncCommand]}};
% Fail safe when Payload is unknown
xmlrpc_callback(_State, Payload) ->
    FaultString = lists:flatten(io_lib:format("Unknown call: ~p", [Payload])),
	io:format("Server received: ~p", [Payload]),
    {false, {response, {fault, -1, FaultString}}}.