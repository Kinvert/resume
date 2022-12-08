function [stat endboard] = floodit(board,moves)
clc
if nargin > 0 %Input detected, notifying program
    comp = 1;
    [stat endboard] = init_run(board,moves,comp);
end
if nargin < 2 %input not detected, prompting user
    prompt = {'Board Size 6-12:'};
    def = {'6'};
    answr = inputdlg(prompt,'Size',1,def);
    ara = cell2mat(answr); ara = str2double(ara);
    board = randi(6,ara,ara,'uint8');
    moves = zeros(1,ara,'uint8');
    comp = 0;
        if ara > 12
        outcome = 'YOU LOSE'
        endboard = board;
        stat = 2;
        return
    end
    if ara < 6
        outcome = 'YOU LOSE'
        endboard = board;
        stat = 2;
        return
    end
    [stat endboard] = init_run(board,moves,comp);
end

function [stat endboard] = init_run(board,moves,comp) %Main program
ara = length(board);
boardb = zeros(ara,ara,'uint8'); boardb(1,1) = 1;
image(board); colormap(jet(7));
turn = 1;
if comp == 0
    helpdlg('Click New Color or Pick Old Color to Quit','Directions'); %help
end
while turn < ara * 2 + 1
    image(board);
    if comp == 0 %User Mouse Selection
        [x,y] = ginput(1);
        x = round(x);
        y = round(y);
        answ = board(y,x);
        moves(1,turn) = answ;
        if answ == board(1,1)
            stat = 0;
            outcome = 'You Quit' %#ok<NASGU>
            endboard = board;
            return
        end
    end
    if comp == 1
        if turn < length(moves) %avoiding number outside matrix
        answ = moves(1,turn);
        end
        if turn > length(moves) %see if computer quit
            outcome = 'You Quit' %#ok<NASGU>
            stat = 0;
            endboard = board;
            return
        end
    end
    oldclr = board(1,1); %Color to compare to
    ii = 1; jj = 1; %Start at top left corner
    while ii < ara + 1 %cycle through positions in matrix
        while jj < ara + 1
            if oldclr == board(jj,ii) %Deciding what adjacent blocks change color
                board(jj,ii) = answ;
                boardb(jj,ii) = 1;
                if ii < ara
                    if board(jj,ii+1) == oldclr %check right
                        board(jj,ii+1) = answ;
                        boardb(jj,ii+1) = 1;
                    elseif board(jj,ii+1) == answ
                        boardb(jj,ii+1) = 1;
                    end
                end
                if ii>1
                    if board(jj,ii-1) == oldclr %check left
                        board(jj,ii-1) = answ;
                        boardb(jj,ii-1) = 1;
                    elseif board(jj,ii-1) == answ
                        boardb(jj,ii-1) = 1;
                    end
                end 
                if jj<ara
                    if board(jj+1,ii) == answ %check down
                        boardb(jj+1,ii) = 1;
                    end
                end
            else
                if jj == 1;
                    ii = ii + 13;
                end
                jj = 13; %stop looking in this column
            end
            if jj < ara + 1
                if answ == board(jj,ii)
                    boardb(jj,ii) = 1;
                end
            end
            jj = jj + 1; %next row
        end
        jj = 1; %start at top row
        ii = ii + 1; %next column
    end
repeats = 1;
while repeats < 10 %iterates to be sure all directions from each position are checked
ii=1;jj=1;
while ii < ara + 1
    while jj < ara + 1
        if boardb(jj,ii) == 1
            board(jj,ii) = answ;
            if ii < ara
                if board(jj,ii+1) == answ
                    boardb(jj,ii+1) = 1;
                end
            end
            if ii > 1
                if board(jj,ii-1) == answ
                    boardb(jj,ii-1) = 1;
                end
            end
            if jj < ara
                if board(jj+1,ii) == answ
                    boardb(jj+1,ii) = 1;
                end
            end
            if jj > 1
                if board(jj-1,ii) == answ
                    boardb(jj-1,ii) = 1;
                end
            end
        end
        jj = jj + 1;
    end
    jj = 1; ii = ii + 1;
end
repeats = repeats + 1;
end
turnsleft = ara * 2 - turn
turn = turn + 1;
kk = 1; some = 0;
while kk < ara + 1
some = sum(boardb(:,kk)) + some;
kk = kk + 1;
end
if some == ara * ara %win
    stat = 1;
    outcome = 'YOU WIN'%#ok<NASGU>
    image(board);
    endboard = board;
    return
else %lose
    if turnsleft == 0
    stat = 2;
    outcome = 'You Lose'%#ok<NASGU>
    endboard = board;
    return
    end
end
end

