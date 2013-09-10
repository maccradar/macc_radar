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
-module(astar).
-export([astar/3, astar/2, yen/4]).

-include("states.hrl").
-include_lib("util/include/debug.hrl").
-include_lib("eunit/include/eunit.hrl").

astar(Start, Goal) ->
	modum_proxy:get_id() ! {get_graph, self()},
	receive
		{?reply, get_graph, G} -> astar(G,Start,Goal);
		Other -> io:format("error receiving graph: ~w~n", [Other]), ?undefined
	end.
	
astar(Graph,Start,Goal) ->
  %io:format("astar from ~w to ~w~n",[Start,Goal]),
  Closedset = sets:new(), % The set of nodes already evaluated.
  Openset = sets:add_element(Start,sets:new()), %The set of tentative nodes to be evaluated

  Fscore = dict:append(Start, h_score(Graph,Start,Goal), dict:new()),
  Gscore = dict:append(Start, 0, dict:new()), % Distance from start along optimal path.
  
  CameFrom = dict:append(Start, none, dict:new()),
  Solution = astar_step(Graph, Goal, Closedset, Openset, Fscore, Gscore, CameFrom),
  %io:format("reverse solution: ~w~n",[Solution]),
  case Solution of
	failure -> failure;
	_ -> lists:reverse(Solution)
  end.
  
astar_step(Graph, Goal, Closedset, Openset, Fscore, Gscore, CameFrom) ->
  Size = sets:size(Openset),
  if
     Size =:= 0 ->
      failure;
    true ->
      X = best_step(sets:to_list(Openset), Fscore, none, infinity),
      if
        X =:= Goal ->
          reconstruct_path(CameFrom, Goal);
        true ->
          NextOpen = sets:del_element(X, Openset),
          NextClosed = sets:add_element(X, Closedset),
          Neighbours = neighbour_nodes(Graph,X),
          {NewOpen, NewF, NewG, NewFrom} = scan(Graph, Goal,X, Neighbours, NextOpen, NextClosed, Fscore, Gscore, CameFrom),
          astar_step(Graph,Goal, NextClosed, NewOpen, NewF, NewG, NewFrom)
      end
  end.
          
scan(_Graph, _Goal,_X, [], Open, _Closed, F, G, From) ->
  {Open, F, G, From};
scan(Graph, Goal, X, [Y|N], Open, Closed, F, G, From) ->
  Member = sets:is_element(Y, Closed),
  if
     Member ->
      scan(Graph, Goal, X, N, Open, Closed, F, G, From);
    true ->
      [G0] = dict:fetch(X, G),
      TrialG = G0 + dist_between(Graph,X,Y),
      IsOpen = sets:is_element(Y, Open),
      if
        IsOpen ->
          [OldG] = dict:fetch(Y, G),
          if
            TrialG < OldG ->
              {NewF, NewG, NewFrom} = update(Graph, Goal, X, Y, F, G, From, TrialG),
              scan(Graph, Goal, X, N, Open, Closed, NewF, NewG, NewFrom);
            true ->
              scan(Graph, Goal, X, N, Open, Closed, F, G, From)
            end;
        true ->
          NewOpen = sets:add_element(Y, Open),
          {NewF, NewG, NewFrom} = update(Graph, Goal, X, Y, F, G, From, TrialG),
          scan(Graph, Goal, X, N, NewOpen, Closed, NewF, NewG, NewFrom)
      end
  end.
          
update(Graph, Goal, X, Y, OldF, OldG, OldFrom, GValue) ->
  KeyG = dict:is_key(Y, OldG),
  KeyF = dict:is_key(Y, OldF),
  KeyFrom = dict:is_key(Y, OldFrom),
  if
    KeyG ->
      update(Graph, Goal, X, Y, OldF, dict:erase(Y, OldG), OldFrom, GValue);
    KeyF ->
      update(Graph, Goal, X, Y, dict:erase(Y, OldF), OldG, OldFrom, GValue);
    KeyFrom ->
      update(Graph, Goal, X, Y, OldF, OldG, dict:erase(Y, OldFrom), GValue);
    true ->
      NewFrom = dict:append(Y, X, OldFrom),
      NewG = dict:append(Y, GValue, OldG),
      NewF = dict:append(Y, GValue + h_score(Graph, Y, Goal), OldF), % Estimated total distance from start to goal through y.
      {NewF, NewG, NewFrom}
  end.
     
reconstruct_path(CameFrom, Node) ->
  [Value] = dict:fetch(Node, CameFrom),
  if
    none =:= Value ->
      [Node];
    true ->
      [Node | reconstruct_path(CameFrom, Value)]
  end.

best_step([H|Open], Score, none, _) ->
    [V] = dict:fetch(H, Score),
    best_step(Open, Score, H, V);
best_step([], _Score, Best, _BestValue) ->
     Best;
best_step([H|Open], Score, Best, BestValue) ->
  [Value] = dict:fetch(H, Score),
  if
    Value < BestValue ->
      best_step(Open, Score, H, Value);
    true ->
      best_step(Open, Score, Best, BestValue)
  end.
  
neighbour_nodes(Graph, X) ->
  N = digraph:out_neighbours(Graph,X),
  %io:format("Neighbours of ~w: ~w~n",[X,N]),
  N.  

dist_between(_G,X,X) ->
	0;
dist_between(Graph, X, Y) ->
	D = fun(E) -> {_,_,_,L} = digraph:edge(Graph,E), L end,
	[Dist] = [D(E1) || E1 <- digraph:edges(Graph,X), E2 <- digraph:edges(Graph,Y), E1==E2],
	%io:format("Dist between ~w and ~w: ~w~n",[X,Y,Dist]),
	Dist.
  
h_score(Graph, StartNode, GoalNode) ->
	%io:format("Calculating H score of ~w to goal ~w~n",[StartNode,GoalNode]),
	VS = digraph:vertex(Graph,StartNode),
	S = case VS of
		{_,#nodeState{shape=[Start|_]}} -> Start;
		{_,#linkState{shape=[Start|_]}} -> Start;
		_ -> io:format("unknown start vertex: ~w~n",[VS])
	end,
	VG = digraph:vertex(Graph,GoalNode),
	G = case VG of
		{_,#nodeState{shape=Shape}} -> [Goal | _] = lists:reverse(Shape), Goal;
		{_,#linkState{shape=Shape}} -> [Goal | _] = lists:reverse(Shape), Goal;
		_ -> io:format("unknown goal vertex: ~w~n",[VG])
	end,
	DistFun = fun(P1,P2) -> math:sqrt(math:pow(P1#point.x-P2#point.x,2) + math:pow(P1#point.y-P2#point.y,2)) end,
	H = DistFun(S, G),
	%io:format("H score of ~w to goal ~w: ~w~n",[StartNode,GoalNode,H]),
	H.

path_length(Graph,[First | Tail]) ->
	{_First, Length} = lists:foldl( fun(T,{F,TD})-> {T,dist_between(Graph,F,T)+TD} end,{First,0},Tail),
	Length.


check_paths(Graph,Paths) ->
	Z = lists:zip([path_length(Graph,P) || P <- Paths], Paths),
	Z =:= lists:sort(Z) orelse io:format("K Paths are not in increasing order: ~w~n", [Z]),
	lists:sort(Z).
	
yen(Graph, Source, Sink, K) ->
	% Determine the shortest path from the source to the sink.
	FirstPath = astar(Graph, Source, Sink),
	Paths = case FirstPath of
		failure -> failure;
		_ -> A = [FirstPath],
			 % Initialize the heap to store the potential kth shortest path.
			 B = [],
			 yen_kloop(Graph, A, B, Source, Sink, 2, K+1)
	end,
	Sorted = check_paths(Graph,Paths).

yen_kloop(_,A,_,_,_,K,K) -> A;
yen_kloop(Graph, A, OldB, Source, Sink, IndexK, K) ->
	% The spur node ranges from the first node to the next to last node in the shortest path.
	PreviousPath = lists:nth(IndexK-1,A),
	B = yen_iloop(Graph, A, OldB, PreviousPath, Sink, 1,length(lists:nth(IndexK-1,A))),
	
	case length(B) of
		0 -> yen_kloop(Graph, A, B, Source, Sink, K, IndexK+1);
		_ -> % Sort the potential k-shortest paths by cost.
			 [{_,NewP} | NewB] = lists:sort(B),
			 % Add the lowest cost path becomes the k-shortest path.
			 yen_kloop(Graph, lists:reverse([NewP | A]),NewB,Source,Sink, IndexK+1,K)
	end.

yen_iloop(_,_,B,_,_,I,I) -> B;
yen_iloop(Graph,A,B,PreviousPath,Sink,IndexI,I) ->
        % Spur node is retrieved from the previous k-shortest path, k - 1.
        SpurNode = lists:nth(IndexI,PreviousPath),
        % The sequence of nodes from the source to the spur node of the previous k-shortest path.
        RootPath = lists:sublist(PreviousPath,IndexI),
		%io:format("PreviousPath: ~w, IndexI: ~w, SpurNode: ~w, RootPath: ~w~n", [PreviousPath,IndexI,SpurNode, RootPath]),
        EdgeInfos = lists:flatten(lists:map(fun(P)->yen_ploop(Graph,P,RootPath,IndexI) end, A)),
        % Calculate the spur path from the spur node to the sink.
        SpurPath = astar(Graph, SpurNode, Sink),
		%io:format("SpurPath: ~w~n", [SpurPath]),
		% Add back the edges that were removed from the graph.
        lists:foreach(fun({E,V1,V2,L})->digraph:add_edge(Graph,E,V1,V2,L) end,EdgeInfos),
		case SpurPath of
			failure -> yen_iloop(Graph, A, B, PreviousPath, Sink, IndexI+1, I);
			_ -> % Entire path is made up of the root path and spur path.
				 [_ | RevRootPath] = lists:reverse(RootPath),
				 TotalPath = lists:append(lists:reverse(RevRootPath),SpurPath),
				 %io:format("TotalPath: ~w~n",[TotalPath]),
				 % Add the potential k-shortest path to the heap.
				 yen_iloop(Graph, A, lists:reverse([{length(TotalPath),TotalPath} | B]), PreviousPath, Sink, IndexI+1, I)
		end.
yen_ploop(Graph, P, RootPath, I) ->
	SubPath = lists:sublist(P, I),
	%io:format("subpath of ~w until ~w: ~w~n",[P,I,SubPath]),
	case RootPath of
		SubPath ->
			Edges = [E1 || E1 <- digraph:edges(Graph,lists:nth(I,P)), E2 <- digraph:edges(Graph,lists:nth(I+1,P)), E1==E2],
			EdgeInfos = [digraph:edge(Graph,E1) || E1 <- Edges],
			% Remove the edges that are part of the previous shortest paths which share the same root path.
			%io:format("edges to remove in ~w: ~w~n",[Graph,Edges]),
            digraph:del_edges(Graph,Edges),
			EdgeInfos;
		_ ->
			[]
	end.
