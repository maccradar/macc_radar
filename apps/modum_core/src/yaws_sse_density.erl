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
-module(yaws_sse_density).
-behaviour(gen_server).

-include_lib("yaws/include/yaws_api.hrl").

%% API
-export([out/1]).

%% gen_server callbacks
-export([init/1, handle_call/3, handle_cast/2, handle_info/2,
         terminate/2, code_change/3]).

-record(state, {
          sock,
          yaws_pid,
          timer,
		  road_types
         }).

out(A) ->
    case (A#arg.req)#http_request.method of
        'GET' ->
            case yaws_api:get_header(A#arg.headers, accept) of
                undefined ->
                    {status, 406};
                Accept ->
                    case string:str(Accept, "text/event-stream") of
                        0 ->
                            {status, 406};
                        _ ->
                            {ok, Pid} = gen_server:start(?MODULE, [A], []),
                            yaws_sse:headers(Pid)
                    end
            end;
        _ ->
            [{status, 405},
             {header, {"Allow", "GET"}}]
    end.

init([Arg]) ->
    process_flag(trap_exit, true),	
	{ok, #state{sock=Arg#arg.clisock, road_types=yaws_api:parse_query(Arg)}}.

handle_call(_Request, _From, State) ->
    {reply, ok, State}.

handle_cast(_Msg, State) ->
    {noreply, State}.

handle_info({ok, YawsPid}, State) ->
    {ok, Timer} = timer:send_after(1, self(), tick),
	% {ok, Timer} = timer:send_interval(300000, self(), tick), % send history update every five minutes
    {noreply, State#state{yaws_pid=YawsPid, timer=Timer}};
handle_info({discard, _YawsPid}, State) ->
    %% nothing to do
    {stop, normal, State};
handle_info(tick, #state{sock=Socket, road_types=RoadTypes}=State) ->
	Roads = [R || {R,"1"} <- RoadTypes],
    io:format("Sending density for links of type ~w~n",[Roads]),
	LinkIds = lists:flatten(lists:map(fun(Road) -> modum_proxy:get_links_of_type(Road) end,Roads)),
	F = fun(LinkId) ->
		LinkId ! {get_density, current, discrete, self()},
		Data = 
			receive
				{density, LinkId, {Coordinates,Density}} -> 	
					[[[L1,L2] || {L1,L2} <- Coordinates],Density]					
			after 5000 -> io:format("Did not receive density for link ~w!~n", [LinkId]), undefined
			end,
		Data
	end,
	Densities = lists:map(F, LinkIds),
	P = base64:encode_to_string(lists:flatten(io_lib:format("~p",[Densities]))),
	Data = yaws_sse:data(P),
	case yaws_sse:send_events(Socket, Data) of
        ok ->
            {noreply, State};
        {error, closed} ->
			io:format("error closed~n"),
            {stop, normal, State};
        {error, Reason} ->
			io:format("error reason: ~w~n",[Reason]),
            {stop, Reason, State}
    end;
handle_info({tcp_closed, _}, State) ->
	io:format("tcp_closed~n",[]),
    {stop, normal, State#state{sock=closed}};
handle_info(Info, State) ->
	io:format("unknown message: ~w~n",[Info]),
    {noreply, State}.

terminate(Reason, #state{sock=Socket, yaws_pid=YawsPid, timer=Timer}) ->
    io:format("Terminate request with reason:~w~n", [Reason]),
	case Timer of
        undefined ->
            ok;
        _ ->
            timer:cancel(Timer)
    end,
    yaws_api:stream_process_end(Socket, YawsPid),
    ok.

code_change(_OldVsn, State, _Extra) ->
    {ok, State}.