//#include "Pixy2.h"
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <stdbool.h>
#include <string.h>
#include "ttt_game.h"
#include "actuator_driver.h"
#include "audio_driver.h"
#include "pixy_driver.h"
#include "lcd_driver.h"
#include "motor_driver.h"

extern uint32_t current_audio_state;

void mainApp(struct Plotter* plotter, UART_HandleTypeDef* phuart){
	int square_taken[9];
	int game_initialized = false;
	int game_over = false;
	int remaining_squares = 9;
	int player_wins = 0;
	int player_losses = 0;
	int draws = 0;
	int mode = 0;
	LCD_CreateButton("Start Game", &current_audio_state);

    /*player 1 turn
     human will draw on square
     camera will detect block
     square is determined by pixel coordinates and marked
     */
    while(!game_over){
        //initialize new or next game
        if(!game_initialized){
        	LCD_Display_Text("Difficulty", 0.1, 100);
        	// remaining_squares is just a dummy
        	mode = LCD_CreateOptions("Easy", "Hard", &remaining_squares);
        	LCD_DrawBoard();
        	Motor_DrawBoard(plotter);
            remaining_squares = 9;
            game_initialized = 1;
            memset(square_taken, 0, sizeof(square_taken)); // set all squares to untaken
        }//end if

		LCD_StartCountdown(8);
		int prev_state[9];
		memcpy(prev_state, square_taken, sizeof(square_taken));
		while(!memcmp(prev_state, square_taken, sizeof(square_taken))) { // keep looping till human has made a move
			Pixy_UpdateBoard(phuart, square_taken); 
		}
		current_audio_state = PLAYER_TURN; // audio state is now in END of Player turn

		int idx = determine_square_idx(prev_state, square_taken);
		if (idx == 9) {
			printf("Catastrophic error, what happened here?");
		}

		//mark square as taken, decrement total squares
		printf("Human takes square %d.\n",idx); // TODO: Remove After Debug
		remaining_squares -= 1;
		LCD_DrawO(idx); // update the new status of the board on the LCD
		print_board(square_taken);
        
        //check for win
        int winner = check_for_win(square_taken); // winner = 1 if human won, winner = 2 if computer won

        //if player 1 didnt win, continue
        if (!winner && remaining_squares){
            //select index of first unclaimed square
            printf("Computer is choosing.\n"); // TODO: Remove After Debug

            int modified_board[9];
            memcpy(modified_board, square_taken, sizeof(square_taken));
            for (int i = 0; i < 9; ++i)
            	modified_board[i] = square_taken[i] ? (square_taken[i] == 1 ? -1 : 1) : 0; // change the board implementation to make it compatible with the minimax algo (-1 is player and 1 is computer now)

	    // vary the difficulty depending on configured difficulty settings	
            if (mode == 1)
            	idx = rand() % 10 < 8 ? easyAI(modified_board) : impossibleAI(modified_board);
            else
            	idx = rand() % 10 < 2 ? easyAI(modified_board) : impossibleAI(modified_board);

            square_taken[idx] = 2; // mark computer selected square as taken
            remaining_squares -= 1;
            Motor_Move_Square(plotter, idx);
            HAL_Delay(1000);
            Motor_DrawX(plotter, 250);
            LCD_DrawX(idx);
            HAL_Delay(1000);
            Motor_Reset(plotter);
            
            printf("Computer takes square %d.\n",idx); // TODO: Remove After Debug
            print_board(square_taken);
            
            winner = check_for_win(square_taken);
        }
        
        //if winner, declare winner and end game
        if(winner){
            game_over = true;
            //returns 1 for player 1 win, 2 for player 2 win
            printf("Player %d wins!\n",winner); // TODO: Remove After Debug
            if (winner == 1) {
            	player_wins++;
            	current_audio_state = PLAYER_WIN;
            	LCD_Display_Text("You win!", 0.5, 2000);
            	LCD_Clear();
            } else {
            	player_losses++;
            	current_audio_state = PLAYER_LOSE;
            	LCD_Display_Text("You lose.", 0.5, 2000);
            	LCD_Clear();
            }
        } else if (!remaining_squares){
            printf("No winner!\n"); // TODO: Remove After Debug
            draws++;
            current_audio_state = PLAYER_DRAW;
        	LCD_Display_Text("Draw", 0.5, 2000);
        	LCD_Clear();
            game_over = true;
        }
        
        if(game_over){
        	int new_game = LCD_CreateOptions("Play again?", "End Game", &current_audio_state);
            //prompt user for new game
            if(new_game == 1){
                game_initialized = false;
                game_over = false;
            } else {
            	LCD_Display_Text("Thanks for playing!", 0.4, 1000);
            	char scores[10];
            	itoa(player_wins, scores, 10);
            	char buf[10];
            	strcat(scores, "-");
            	itoa(player_losses, buf, 10);
            	strcat(scores, buf);
            	strcat(scores, "-");
            	itoa(draws, buf, 10);
            	strcat(scores, buf);
            	LCD_Display_Text(scores, 0.6, 5000); //show the running scoreboard of games won,loss and tied
            } //end  if
        }//end  if
    }//end while
}//end main

// find index of the newly occupied square
int determine_square_idx(int* prev_state, int* new_state){
	for (int i = 0; i < 9; ++i) {
		if (prev_state[i] != new_state[i])
			return i;
	}
	return 9;
}//end determine_square_idx

void print_prompt(void){
    
    printf("Enter the number of a square from this grid.\n\n");

    printf(" 0 | 1 | 2 \n");
    printf("__________\n");
    printf(" 3 | 4 | 5 \n");
    printf("__________\n");
    printf(" 6 | 7 | 8 \n\n");
}

void print_board(int* square_taken){
    printf(" %c | %c | %c \n", get_xo(square_taken[0]),get_xo(square_taken[1]),get_xo(square_taken[2]));
    printf("__________\n");
    printf(" %c | %c | %c \n", get_xo(square_taken[3]),get_xo(square_taken[4]),get_xo(square_taken[5]));
    printf("__________\n");
    printf(" %c | %c | %c \n", get_xo(square_taken[6]),get_xo(square_taken[7]),get_xo(square_taken[8]));
}//end print_board definition

char get_xo(int num){
    if(num == 1){
        return 'x';
    }//end if
    if(num == 2){
        return 'o';
    }//end if
    return ' ';
}//end get_xo definition


int check_for_win(int* square_taken){
    //Check for horizontal win
    if (square_taken[0] == square_taken[1] && square_taken[1] == square_taken[2] &&
        square_taken[0] != 0 && square_taken[1] != 0 &&  square_taken[2] != 0) {
        return square_taken[0];
    }//end if
    else if(square_taken[3] == square_taken[4] && square_taken[4] == square_taken[5] &&
            square_taken[3] != 0 && square_taken[4] != 0 &&  square_taken[5] != 0){
        return square_taken[3];
    }
    else if(square_taken[6] == square_taken[7] && square_taken[7] == square_taken[8] &&
            square_taken[6] != 0 && square_taken[7] != 0 &&  square_taken[8] != 0){
        return square_taken[6];
    }

    //Check for vertical win
    if (square_taken[0] == square_taken[3] && square_taken[3] == square_taken[6] &&
        square_taken[0] != 0 && square_taken[3] != 0 &&  square_taken[6] != 0) {
        return square_taken[0];
    }
    else if(square_taken[1] == square_taken[4] && square_taken[4] == square_taken[7] &&
            square_taken[1] != 0 && square_taken[4] != 0 &&  square_taken[7] != 0){
        return square_taken[1];
    }
    else if(square_taken[2] == square_taken[5] && square_taken[5] == square_taken[8] &&
            square_taken[2] != 0 && square_taken[5] != 0 &&  square_taken[8] != 0){
        return square_taken[2];
    }
    
    // Checking for diagonal win.
    if (square_taken[0] == square_taken[4] && square_taken[4] == square_taken[8] &&
        square_taken[0] != 0 && square_taken[4] != 0 &&  square_taken[8] != 0) {
        return square_taken[0];
    }
    else if(square_taken[2] == square_taken[4] && square_taken[4] == square_taken[6] &&
            square_taken[2] != 0 && square_taken[4] != 0 &&  square_taken[6] != 0){
        return square_taken[2];
    }
      
    // Else if no winner, return 0
    return 0;
}// end check_for_win definition

int easyAI(int* square_taken) {
    // randomly choose square
	int i = rand() % 9;
	while (square_taken[i])
		i = rand() % 9;
	return i;
}
// for every possible move the computer can make, find the move that produces the worst board for the player
int impossibleAI(int* square_taken) {
    int move = -1;
    int score = -2;
    int i;
    for(i = 0; i < 9; ++i) {
        if(square_taken[i] == 0) {
        	square_taken[i] = 1; // simulate computer takes this square
            int tempScore = -minimax(square_taken, -1); // see if this creates a worse board for player
            square_taken[i] = 0;
            if(tempScore > score) {
                score = tempScore;
                move = i;
            }
        }
    }
    //returns a score based on minimax tree at a given node.
    return move;
}

int minimax(int square_taken[9], int player) {
    //How is the position like for player (their turn) on board?
    int winner = check_for_win(square_taken);
    if(winner != 0) return winner*player;

    int move = -1;
    int score = -2;//Losing moves are preferred to no move
    int i;
    for(i = 0; i < 9; ++i) {//For all moves,
        if(square_taken[i] == 0) {//If legal,
            square_taken[i] = player;//Try the move
            int thisScore = -minimax(square_taken, player*-1);
            if(thisScore > score) {
                score = thisScore;
                move = i;
            }//Pick the one that's worst for the opponent
            square_taken[i] = 0;//Reset board after try
        }
    }
    if(move == -1) return 0; // return if all squares on board are full
    return score;
}
