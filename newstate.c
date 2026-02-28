// Vu H 2026 



#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <limits.h>
#include <string.h>

#define SIZE 12 // MAX Maze Length
#define LENGTH 7
#define HEIGHT 11

#define BITS 15          // 4 bits/walls around each cell
#define TOT_CELLS (HEIGHT * LENGTH) // Total number of cells to be found
#define TOT_DEST_CELLS 1 // Number of destination cells
#define Y_DEST 9         // Destination y-coordinate
#define X_DEST 3         // Destination x-coordinate
#define LAST_VIS_CELLS 4 // Last temporary visited cells

// Walls
#define NORTH_WALL 0b1000 // 8
#define EAST_WALL 0b0100  // 4
#define SOUTH_WALL 0b0010 // 2
#define WEST_WALL 0b0001  // 1

#define VERT_WALLS (NORTH_WALL | SOUTH_WALL)
#define HORIZ_WALLS (WEST_WALL | EAST_WALL)

// Cells Checking
#define CELL_VISITED 0b10000 // 16
#define CELL_USED 0b100000   // 32 - Cell used for the final route
#define CELL_OUT 0b1000000   // 64 - Out of Bound
#define DEST_CELL 0b10000000 // 128 - Destination cells

// Mouse Pointing Direction
#define NORTH 0
#define EAST 1
#define SOUTH 2
#define WEST 3

static const int dir_dx[4] = {0, 1, 0, -1};
static const int dir_dy[4] = {1, 0, -1, 0};
static const unsigned int dir_wall_mask[4] = {NORTH_WALL, EAST_WALL, SOUTH_WALL, WEST_WALL};

#define ANSI_RESET "\033[0m"
#define ANSI_RED "\033[31m"
#define ANSI_GREEN "\033[32m"
#define ANSI_YELLOW "\033[33m"
#define ANSI_CYAN "\033[36m"

static int In_Physical_Bounds(int x, int y)
{
    return (x >= 0 && x < LENGTH && y >= 0 && y < HEIGHT);
}

static void Remove_Wall_Bit(unsigned int *cell, unsigned int wall_bit)
{
    if ((*cell & wall_bit) == wall_bit)
        *cell ^= wall_bit;
}

static void Add_Wall_Bit(unsigned int *cell, unsigned int wall_bit)
{
    *cell |= wall_bit;
}

static void Open_Passage(unsigned int maze[HEIGHT][LENGTH], int x1, int y1, int x2, int y2)
{
    if (!In_Physical_Bounds(x1, y1) || !In_Physical_Bounds(x2, y2))
        return;

    if ((x2 == x1 + 1) && (y2 == y1))
    {
        Remove_Wall_Bit(&maze[y1][x1], EAST_WALL);
        Remove_Wall_Bit(&maze[y2][x2], WEST_WALL);
    }
    else if ((x2 == x1 - 1) && (y2 == y1))
    {
        Remove_Wall_Bit(&maze[y1][x1], WEST_WALL);
        Remove_Wall_Bit(&maze[y2][x2], EAST_WALL);
    }
    else if ((x2 == x1) && (y2 == y1 + 1))
    {
        Remove_Wall_Bit(&maze[y1][x1], NORTH_WALL);
        Remove_Wall_Bit(&maze[y2][x2], SOUTH_WALL);
    }
    else if ((x2 == x1) && (y2 == y1 - 1))
    {
        Remove_Wall_Bit(&maze[y1][x1], SOUTH_WALL);
        Remove_Wall_Bit(&maze[y2][x2], NORTH_WALL);
    }
}

static void Close_Passage(unsigned int maze[HEIGHT][LENGTH], int x1, int y1, int x2, int y2)
{
    if (!In_Physical_Bounds(x1, y1) || !In_Physical_Bounds(x2, y2))
        return;

    if ((x2 == x1 + 1) && (y2 == y1))
    {
        Add_Wall_Bit(&maze[y1][x1], EAST_WALL);
        Add_Wall_Bit(&maze[y2][x2], WEST_WALL);
    }
    else if ((x2 == x1 - 1) && (y2 == y1))
    {
        Add_Wall_Bit(&maze[y1][x1], WEST_WALL);
        Add_Wall_Bit(&maze[y2][x2], EAST_WALL);
    }
    else if ((x2 == x1) && (y2 == y1 + 1))
    {
        Add_Wall_Bit(&maze[y1][x1], NORTH_WALL);
        Add_Wall_Bit(&maze[y2][x2], SOUTH_WALL);
    }
    else if ((x2 == x1) && (y2 == y1 - 1))
    {
        Add_Wall_Bit(&maze[y1][x1], SOUTH_WALL);
        Add_Wall_Bit(&maze[y2][x2], NORTH_WALL);
    }
}

// Generating a maze to simulate an environement
void Generate_Maze(unsigned int maze[HEIGHT][LENGTH])
{
    unsigned int r, c;

    for (r = 0; r < HEIGHT; r++)
        for (c = 0; c < LENGTH; c++)
        {
            maze[r][c] = BITS;
        }
}

// Generating the channel map to get to destination
void Generate_Channel_Maze(unsigned int maze[HEIGHT][LENGTH])
{
    int x, y;

    // First open every internal passage.
    for (y = 0; y < HEIGHT; y++)
        for (x = 0; x < (LENGTH - 1); x++)
            Open_Passage(maze, x, y, x + 1, y);

    for (y = 0; y < (HEIGHT - 1); y++)
        for (x = 0; x < LENGTH; x++)
            Open_Passage(maze, x, y, x, y + 1);

    // Start cell keeps east closed.
    Close_Passage(maze, 0, 0, 1, 0);

    // Vertical channel separators.
    for (y = 0; y < HEIGHT; y++)
    {
        Close_Passage(maze, 1, y, 2, y);
        Close_Passage(maze, 4, y, 5, y);
    }

    // Crossover gaps.
    Open_Passage(maze, 1, 1, 2, 1);
    Open_Passage(maze, 4, 1, 5, 1);
    Open_Passage(maze, 1, 8, 2, 8);
    Open_Passage(maze, 4, 8, 5, 8);
    Open_Passage(maze, 1, 9, 2, 9);
    Open_Passage(maze, 4, 9, 5, 9);
    Open_Passage(maze, 1, 10, 2, 10);
    Open_Passage(maze, 4, 10, 5, 10);

    // Top horizontal barrier with single center opening.
    for (x = 0; x < LENGTH; x++)
        if (x != 3)
            Close_Passage(maze, x, 8, x, 9);

    // Green gate: upside-down U.
    for (x = 2; x <= 4; x++)
        Close_Passage(maze, x, 5, x, 6);
    Close_Passage(maze, 1, 5, 2, 5);
    Close_Passage(maze, 4, 5, 5, 5);
}

// Show the "values" of the walls
void Print_Maze(unsigned int maze[HEIGHT][LENGTH])
{
    int r, c;

    printf("--- Generated Maze ----\n");
    for (r = (HEIGHT - 1); r > -1; r--)
    {
        for (c = 0; c < LENGTH; c++)
        {
            printf("%d\t", maze[r][c]);
        }
        printf("\n");
    }
    printf("\n\n");
}

void Print_Mouse_Maze(unsigned int mouse_maze[SIZE][SIZE])
{
    int r, c, cell_value;

    printf("--- Maze Known by the Mouse ---\n");
    for (r = (SIZE - 1); r > -1; r--)
    {
        for (c = 0; c < SIZE; c++)
        {
            cell_value = mouse_maze[r][c];

            // we want to see just the walls
            // OR if the cell is OUT OF BOUNDS
            if ((cell_value & CELL_OUT) == CELL_OUT)
                cell_value = CELL_OUT;
            else
            {
                if ((cell_value & DEST_CELL) == DEST_CELL)
                    cell_value ^= DEST_CELL;
                if ((cell_value & CELL_VISITED) == CELL_VISITED)
                    cell_value ^= CELL_VISITED;
                if ((cell_value & CELL_USED) == CELL_USED)
                    cell_value ^= CELL_USED;
            }

            printf("%d\t", cell_value);
        }
        printf("\n");
    }
    printf("\n\n");

    printf("--- Final Route to Destination ---\n");
    for (r = (SIZE - 1); r > -1; r--)
    {
        for (c = 0; c < SIZE; c++)
        {
            if ((mouse_maze[r][c] & CELL_USED) == CELL_USED)
                printf("1");
            else
                printf("0");
            printf("\t");
        }
        printf("\n");
    }
}

static int Use_Ansi_Colours(void)
{
    const char *no_color = getenv("NO_COLOR");
    if ((no_color != NULL) && (no_color[0] != '\0'))
        return 0;
    return 1;
}

static const char *North_Wall_Colour(int row, int col)
{
    if (row == (HEIGHT - 1))
        return ANSI_RED;
    if ((row == 8) && (col != 3))
        return ANSI_YELLOW;
    if ((row == 5) && (col >= 2) && (col <= 4))
        return ANSI_GREEN;

    return NULL;
}

static const char *West_Wall_Colour(int row, int col)
{
    if (col == 0)
        return ANSI_RED;

    if (((col == 2) || (col == 5)) && (row != 1) && (row != 8))
        return ANSI_GREEN;

    return NULL;
}

static void Print_Segment(const char *segment, const char *colour, int use_colour)
{
    if (use_colour && (colour != NULL))
        printf("%s%s%s", colour, segment, ANSI_RESET);
    else
        printf("%s", segment);
}

void Print_ASCII_Walls(unsigned int maze[HEIGHT][LENGTH])
{
    int r, c;
    int use_colour;
    const char *north_colour;
    const char *west_colour;

    use_colour = Use_Ansi_Colours();

    printf("\n--- ASCII Maze ---\n");
    printf("Legend: S=start, D=destination\n");

    for (r = (HEIGHT - 1); r > -1; r--)
    {
        for (c = 0; c < LENGTH; c++)
        {
            printf("+");
            if ((maze[r][c] & NORTH_WALL) == NORTH_WALL)
            {
                north_colour = North_Wall_Colour(r, c);
                Print_Segment("---", north_colour, use_colour);
            }
            else
                printf("   ");
        }
        printf("+\n");

        for (c = 0; c < LENGTH; c++)
        {
            if ((maze[r][c] & WEST_WALL) == WEST_WALL)
            {
                west_colour = West_Wall_Colour(r, c);
                Print_Segment("|", west_colour, use_colour);
            }
            else
                printf(" ");

            if ((r == 0) && (c == 0))
                printf(" S ");
            else if ((r == Y_DEST) && (c == X_DEST))
            {
                if (use_colour)
                    printf(" %sD%s ", ANSI_CYAN, ANSI_RESET);
                else
                    printf(" D ");
            }
            else
                printf("   ");
        }

        if ((maze[r][LENGTH - 1] & EAST_WALL) == EAST_WALL)
            Print_Segment("|", ANSI_RED, use_colour);
        else
            printf(" ");
        printf("\n");
    }

    for (c = 0; c < LENGTH; c++)
    {
        printf("+");
        if ((maze[0][c] & SOUTH_WALL) == SOUTH_WALL)
            Print_Segment("---", ANSI_RED, use_colour);
        else
            printf("   ");
    }
    printf("+\n\n");
}

void Print_ASCII_Maze(unsigned int maze[HEIGHT][LENGTH], unsigned int mouse_maze[SIZE][SIZE])
{
    int r, c;
    char marker;
    int use_colour;
    const char *north_colour;
    const char *west_colour;
    const char *marker_colour;

    use_colour = Use_Ansi_Colours();

    printf("\n--- ASCII Maze (Walls + Solved Path) ---\n");
    printf("Legend: S=start, D=destination, *=solved path\n");

    for (r = (HEIGHT - 1); r > -1; r--)
    {
        // Print north walls for this row
        for (c = 0; c < LENGTH; c++)
        {
            printf("+");
            if ((maze[r][c] & NORTH_WALL) == NORTH_WALL)
            {
                north_colour = North_Wall_Colour(r, c);
                Print_Segment("---", north_colour, use_colour);
            }
            else
                printf("   ");
        }
        printf("+\n");

        // Print west/east walls and cell markers
        for (c = 0; c < LENGTH; c++)
        {
            if ((maze[r][c] & WEST_WALL) == WEST_WALL)
            {
                west_colour = West_Wall_Colour(r, c);
                Print_Segment("|", west_colour, use_colour);
            }
            else
                printf(" ");

            marker = ' ';
            marker_colour = NULL;
            if ((r == 0) && (c == 0))
                marker = 'S';
            else if ((mouse_maze[r][c] & DEST_CELL) == DEST_CELL)
            {
                marker = 'D';
                marker_colour = ANSI_CYAN;
            }
            else if ((mouse_maze[r][c] & CELL_USED) == CELL_USED)
                marker = '*';

            if (use_colour && (marker_colour != NULL))
                printf(" %s%c%s ", marker_colour, marker, ANSI_RESET);
            else
                printf(" %c ", marker);
        }

        if ((maze[r][LENGTH - 1] & EAST_WALL) == EAST_WALL)
            Print_Segment("|", ANSI_RED, use_colour);
        else
            printf(" ");
        printf("\n");
    }

    // Print bottom boundary
    for (c = 0; c < LENGTH; c++)
    {
        printf("+");
        if ((maze[0][c] & SOUTH_WALL) == SOUTH_WALL)
            Print_Segment("---", ANSI_RED, use_colour);
        else
            printf("   ");
    }
    printf("+\n\n");
}

struct Mouse_Settings
{
    unsigned int pos_x, pos_y;   // Current position and orientation
    unsigned int dirs[4], m_dir; // 4 possible directions of the mouse
    unsigned int sides_found, dest_cells_found, destination_found, cells_found;
    unsigned int prev_cells[SIZE][SIZE], prev_cell, prev_cell_x, prev_cell_y; // where the last # visited cells will be stored
};

void Mouse_Setup(struct Mouse_Settings *p_mouse)
{
    unsigned int r, c;

    // Mouse Starting position
    p_mouse->pos_x = 0;
    p_mouse->pos_y = 0;

    // Possible directions referred to walls
    p_mouse->dirs[0] = NORTH_WALL;
    p_mouse->dirs[1] = EAST_WALL;
    p_mouse->dirs[2] = SOUTH_WALL;
    p_mouse->dirs[3] = WEST_WALL;
    // Poiting Upwards/North
    p_mouse->m_dir = NORTH;

    // Deterministic dimensions, no side-discovery step required.
    p_mouse->sides_found = 1;
    p_mouse->dest_cells_found = 0;
    p_mouse->destination_found = 0;
    p_mouse->cells_found = 1; // Mouse knows the first cell

    // Clearing -> Cells previously visited
    for (r = 0; r < SIZE; r++)
        for (c = 0; c < SIZE; c++)
            p_mouse->prev_cells[r][c] = 0;

    p_mouse->prev_cell = 1; // This will represent the first of the last 8 visited cells
    p_mouse->prev_cell_x = 99;
    p_mouse->prev_cell_y = 99;
    p_mouse->prev_cells[0][0] = p_mouse->prev_cell; // Starting position
}

void Mouse_Maze_Setup(unsigned int mouse_maze[SIZE][SIZE])
{
    unsigned int r, c;

    for (r = 0; r < SIZE; r++)
        for (c = 0; c < SIZE; c++)
        {
            mouse_maze[r][c] = 0;

            if ((r >= HEIGHT) || (c >= LENGTH))
            {
                mouse_maze[r][c] = CELL_OUT;
                continue;
            }

            if (r == 0)
                mouse_maze[r][c] |= SOUTH_WALL;
            else if (r == (HEIGHT - 1))
                mouse_maze[r][c] |= NORTH_WALL;
            if (c == 0)
                mouse_maze[r][c] |= WEST_WALL;
            else if (c == (LENGTH - 1))
                mouse_maze[r][c] |= EAST_WALL;
        }

    // Setting first cell of the maze (3 mandatory walls)
    mouse_maze[0][0] |= EAST_WALL;
    mouse_maze[0][0] |= CELL_VISITED;
    if (LENGTH > 1)
        mouse_maze[0][1] |= WEST_WALL; // Caused by the setup of the first cell

    // Setting Main destination cell
    mouse_maze[Y_DEST][X_DEST] |= DEST_CELL;
}

static void Next_Cell(unsigned int pos_x, unsigned int pos_y, unsigned int dir, int *next_x, int *next_y)
{
    *next_x = (int)pos_x + dir_dx[dir];
    *next_y = (int)pos_y + dir_dy[dir];
}

static int Can_Move(unsigned int mouse_maze[SIZE][SIZE], unsigned int pos_x, unsigned int pos_y, unsigned int dir)
{
    int next_x, next_y;

    if ((mouse_maze[pos_y][pos_x] & dir_wall_mask[dir]) == dir_wall_mask[dir])
        return 0;

    Next_Cell(pos_x, pos_y, dir, &next_x, &next_y);

    if ((next_x < 0) || (next_x >= SIZE) || (next_y < 0) || (next_y >= SIZE))
        return 0;

    if ((mouse_maze[next_y][next_x] & CELL_OUT) == CELL_OUT)
        return 0;

    return 1;
}

static int Is_Unexplored_Target(unsigned int mouse_maze[SIZE][SIZE], struct Mouse_Settings *p_mouse, int next_x, int next_y)
{
    if ((mouse_maze[next_y][next_x] & CELL_VISITED) != CELL_VISITED)
        return 1;

    // Escape any destination loop by allowing movement away from the single destination anchor.
    if (
        (p_mouse->pos_x == X_DEST) &&
        (p_mouse->pos_y == Y_DEST) &&
        !((next_x == X_DEST) && (next_y == Y_DEST)))
        return 1;

    return 0;
}

static int Choose_Return_Direction(unsigned int mouse_maze[SIZE][SIZE], unsigned int discovered_cells[SIZE][SIZE], struct Mouse_Settings *p_mouse)
{
    unsigned int dir;
    int next_x, next_y;
    int best_dir = -1;
    unsigned int best_val = discovered_cells[p_mouse->pos_y][p_mouse->pos_x];

    for (dir = 0; dir < 4; dir++)
    {
        if (!Can_Move(mouse_maze, p_mouse->pos_x, p_mouse->pos_y, dir))
            continue;

        Next_Cell(p_mouse->pos_x, p_mouse->pos_y, dir, &next_x, &next_y);
        if ((discovered_cells[next_y][next_x] > 0) && (discovered_cells[next_y][next_x] < best_val))
        {
            best_val = discovered_cells[next_y][next_x];
            best_dir = (int)dir;
        }
    }

    return best_dir;
}

static int Choose_Exploration_Fallback(unsigned int mouse_maze[SIZE][SIZE], unsigned int discovered_cells[SIZE][SIZE], struct Mouse_Settings *p_mouse)
{
    unsigned int i;
    int next_x, next_y;
    int best_any_dir = -1;
    int best_nonprev_dir = -1;
    unsigned int best_any_value = UINT_MAX;
    unsigned int best_nonprev_value = UINT_MAX;

    for (i = 0; i < 4; i++)
    {
        unsigned int dir = (p_mouse->m_dir + i) % 4;
        unsigned int neigh_value;
        int is_prev_cell;

        if (!Can_Move(mouse_maze, p_mouse->pos_x, p_mouse->pos_y, dir))
            continue;

        Next_Cell(p_mouse->pos_x, p_mouse->pos_y, dir, &next_x, &next_y);
        neigh_value = discovered_cells[next_y][next_x];
        if (neigh_value == 0)
            continue;

        if (neigh_value < best_any_value)
        {
            best_any_value = neigh_value;
            best_any_dir = (int)dir;
        }

        is_prev_cell = (p_mouse->prev_cell_x == (unsigned int)next_x) && (p_mouse->prev_cell_y == (unsigned int)next_y);
        if (!is_prev_cell && (neigh_value < best_nonprev_value))
        {
            best_nonprev_value = neigh_value;
            best_nonprev_dir = (int)dir;
        }
    }

    if (best_nonprev_dir != -1)
        return best_nonprev_dir;
    return best_any_dir;
}

static int Choose_Path_To_Unvisited(unsigned int mouse_maze[SIZE][SIZE], struct Mouse_Settings *p_mouse)
{
    int queue_x[SIZE * SIZE];
    int queue_y[SIZE * SIZE];
    int first_dir[SIZE][SIZE];
    int seen[SIZE][SIZE];
    int head = 0;
    int tail = 0;
    int sx = (int)p_mouse->pos_x;
    int sy = (int)p_mouse->pos_y;
    int prev_dir = -1;
    int r, c;
    int i;
    unsigned int dir_order[4];

    for (i = 0; i < 4; i++)
    {
        int tx, ty;

        dir_order[i] = (p_mouse->m_dir + (unsigned int)i) % 4;
        Next_Cell((unsigned int)sx, (unsigned int)sy, dir_order[i], &tx, &ty);
        if ((tx == (int)p_mouse->prev_cell_x) && (ty == (int)p_mouse->prev_cell_y))
            prev_dir = (int)dir_order[i];
    }

    if (prev_dir != -1)
    {
        int idx = -1;
        for (i = 0; i < 4; i++)
            if ((int)dir_order[i] == prev_dir)
            {
                idx = i;
                break;
            }

        if ((idx >= 0) && (idx < 3))
        {
            unsigned int prev_saved = dir_order[idx];
            for (i = idx; i < 3; i++)
                dir_order[i] = dir_order[i + 1];
            dir_order[3] = prev_saved;
        }
    }

    for (r = 0; r < SIZE; r++)
        for (c = 0; c < SIZE; c++)
        {
            seen[r][c] = 0;
            first_dir[r][c] = -1;
        }

    seen[sy][sx] = 1;
    queue_x[tail] = sx;
    queue_y[tail] = sy;
    tail += 1;

    while (head < tail)
    {
        int cur_x = queue_x[head];
        int cur_y = queue_y[head];
        head += 1;

        if (!((cur_x == sx) && (cur_y == sy)) &&
            ((mouse_maze[cur_y][cur_x] & CELL_OUT) != CELL_OUT) &&
            ((mouse_maze[cur_y][cur_x] & CELL_VISITED) != CELL_VISITED))
            return first_dir[cur_y][cur_x];

        for (i = 0; i < 4; i++)
        {
            unsigned int dir;
            int nx, ny;

            if ((cur_x == sx) && (cur_y == sy))
                dir = dir_order[i];
            else
                dir = (unsigned int)i;

            if (!Can_Move(mouse_maze, (unsigned int)cur_x, (unsigned int)cur_y, dir))
                continue;

            Next_Cell((unsigned int)cur_x, (unsigned int)cur_y, dir, &nx, &ny);
            if (seen[ny][nx] == 1)
                continue;

            seen[ny][nx] = 1;
            if ((cur_x == sx) && (cur_y == sy))
                first_dir[ny][nx] = (int)dir;
            else
                first_dir[ny][nx] = first_dir[cur_y][cur_x];

            queue_x[tail] = nx;
            queue_y[tail] = ny;
            tail += 1;
        }
    }

    return -1;
}

void Mouse_Exploring(unsigned int maze[HEIGHT][LENGTH], unsigned int mouse_maze[SIZE][SIZE], struct Mouse_Settings *p_mouse, unsigned int discovered_cells[SIZE][SIZE])
{
    unsigned int chosen_dir = NORTH, temp_prev_cell, stop = 1;
    int turn_check;

    unsigned int i, r, c;
    int next_x, next_y;

    (void)maze;

    // Find all of the cells - Flood Fill style exploration.
    if (p_mouse->cells_found != TOT_CELLS)
    {
        for (i = 0; i < 4; i++)
        {
            unsigned int temp_dir = (p_mouse->m_dir + i) % 4;

            if (!Can_Move(mouse_maze, p_mouse->pos_x, p_mouse->pos_y, temp_dir))
                continue;

            Next_Cell(p_mouse->pos_x, p_mouse->pos_y, temp_dir, &next_x, &next_y);

            if (Is_Unexplored_Target(mouse_maze, p_mouse, next_x, next_y))
            {
                chosen_dir = temp_dir;
                stop = 0;
                break;
            }
        }

        if (stop == 1)
        {
            int fallback_path = Choose_Path_To_Unvisited(mouse_maze, p_mouse);
            int fallback_alt = -1;
            int fallback_dir = fallback_path;

            if (fallback_dir < 0)
            {
                fallback_alt = Choose_Exploration_Fallback(mouse_maze, discovered_cells, p_mouse);
                fallback_dir = fallback_alt;
            }

            if (fallback_dir >= 0)
            {
                chosen_dir = (unsigned int)fallback_dir;
                stop = 0;
            }
        }
    }

    // If all cells have been found, move back to first cell
    // Follow the lowest number of the discovered cells order
    else if (discovered_cells[p_mouse->pos_y][p_mouse->pos_x] != 1) // Don't move if inside first cell
    {
        int return_dir = Choose_Return_Direction(mouse_maze, discovered_cells, p_mouse);
        if (return_dir >= 0)
        {
            chosen_dir = (unsigned int)return_dir;
            stop = 0;
        }
    }

    if (stop == 0) // move if allowed
    {
        //Turn Right/Left
        // Negative = left; Positive = Right; Zero = No turning
        turn_check = chosen_dir - p_mouse->m_dir;
        if ((turn_check == 3) || (turn_check == -3))
            turn_check /= -3;

        // Simulation purposes - Directly set Mouse Direction
        p_mouse->m_dir = chosen_dir;

        // Storing current cell as previous cell before movement
        p_mouse->prev_cell_x = p_mouse->pos_x;
        p_mouse->prev_cell_y = p_mouse->pos_y;

        // Forward Movement
        p_mouse->pos_x = (unsigned int)((int)p_mouse->pos_x + dir_dx[p_mouse->m_dir]);
        p_mouse->pos_y = (unsigned int)((int)p_mouse->pos_y + dir_dy[p_mouse->m_dir]);

        //--- Checking if the mouse went back to previously visited cells ---
        // Current cell is not part of the last # of visited cells
        if (p_mouse->prev_cells[p_mouse->pos_y][p_mouse->pos_x] == 0)
        {
            if (p_mouse->prev_cell != LAST_VIS_CELLS)
                p_mouse->prev_cell += 1;
            else
                for (r = 0; r < SIZE; r++)
                    for (c = 0; c < SIZE; c++)
                        if (p_mouse->prev_cells[r][c] > 0)
                            p_mouse->prev_cells[r][c] -= 1;

            p_mouse->prev_cells[p_mouse->pos_y][p_mouse->pos_x] = p_mouse->prev_cell;
        }

        // Mouse came bake to one of the # previously visited cells
        else if ((mouse_maze[p_mouse->prev_cell_y][p_mouse->prev_cell_x] & CELL_OUT) != CELL_OUT) // Checking if the mouse's last cell was an a CELL OUT
        {
            temp_prev_cell = p_mouse->prev_cell;
            while (temp_prev_cell > p_mouse->prev_cells[p_mouse->pos_y][p_mouse->pos_x])
            {
                for (r = 0; r < SIZE; r++)
                    for (c = 0; c < SIZE; c++)
                    {
                        if ((r == 0) && (c == 0))
                            continue;
                        else if (p_mouse->prev_cells[r][c] == temp_prev_cell)
                        {
                            if (
                                // North
                                (
                                    ((mouse_maze[r][c] & NORTH_WALL) != NORTH_WALL) &&
                                    ((mouse_maze[r + 1][c] & CELL_OUT) != CELL_OUT) &&
                                    ((mouse_maze[r + 1][c] & CELL_VISITED) != CELL_VISITED))

                                ||

                                // East
                                (
                                    ((mouse_maze[r][c] & EAST_WALL) != EAST_WALL) &&
                                    ((mouse_maze[r][c + 1] & CELL_OUT) != CELL_OUT) &&
                                    ((mouse_maze[r][c + 1] & CELL_VISITED) != CELL_VISITED))

                                ||

                                // South
                                (
                                    ((mouse_maze[r][c] & SOUTH_WALL) != SOUTH_WALL) &&
                                    ((mouse_maze[r - 1][c] & CELL_OUT) != CELL_OUT) &&
                                    ((mouse_maze[r - 1][c] & CELL_VISITED) != CELL_VISITED))

                                ||

                                // West
                                (
                                    ((mouse_maze[r][c] & WEST_WALL) != WEST_WALL) &&
                                    ((mouse_maze[r][c - 1] & CELL_OUT) != CELL_OUT) &&
                                    ((mouse_maze[r][c - 1] & CELL_VISITED) != CELL_VISITED)))
                                break;
                            else if ((mouse_maze[r][c] & DEST_CELL) != DEST_CELL)
                            {
                                // Keep cells traversable in this map to avoid over-pruning channels.
                            }
                        }
                    }
                temp_prev_cell -= 1;
            }

            // Resetting the last # visited cells memory
            for (r = 0; r < SIZE; r++)
                for (c = 0; c < SIZE; c++)
                    p_mouse->prev_cells[r][c] = 0;
            p_mouse->prev_cell = 1;
            p_mouse->prev_cells[p_mouse->pos_y][p_mouse->pos_x] = p_mouse->prev_cell;
        }
    }
}

void Walls_Check(unsigned int maze[HEIGHT][LENGTH], struct Mouse_Settings *p_mouse, unsigned int mouse_maze[SIZE][SIZE])
{
    // Adding walls in the unvisited cells
    if ((mouse_maze[p_mouse->pos_y][p_mouse->pos_x] & CELL_VISITED) != CELL_VISITED) // Check if mouse has already been in current cell
    {

        // North
        if ((maze[p_mouse->pos_y][p_mouse->pos_x] & NORTH_WALL) == NORTH_WALL) // Checking if there is a wall in front
        {
            mouse_maze[p_mouse->pos_y][p_mouse->pos_x] |= NORTH_WALL; // Add top wall
            if (p_mouse->pos_y != (HEIGHT - 1))
                mouse_maze[p_mouse->pos_y + 1][p_mouse->pos_x] |= SOUTH_WALL; // Add back wall to the following row
        }
        // East
        if ((maze[p_mouse->pos_y][p_mouse->pos_x] & EAST_WALL) == EAST_WALL) // Checking if there is a wall on the right
        {
            mouse_maze[p_mouse->pos_y][p_mouse->pos_x] |= EAST_WALL; // Add right wall
            if (p_mouse->pos_x != (LENGTH - 1))
                mouse_maze[p_mouse->pos_y][p_mouse->pos_x + 1] |= WEST_WALL; // Add left wall to the following column
        }
        // South
        if ((maze[p_mouse->pos_y][p_mouse->pos_x] & SOUTH_WALL) == SOUTH_WALL) // Checking if there is a wall below
        {
            mouse_maze[p_mouse->pos_y][p_mouse->pos_x] |= SOUTH_WALL; // Add bottom wall
            if (p_mouse->pos_y != 0)
                mouse_maze[p_mouse->pos_y - 1][p_mouse->pos_x] |= NORTH_WALL; // Add top wall to the previous row
        }
        // West
        if ((maze[p_mouse->pos_y][p_mouse->pos_x] & WEST_WALL) == WEST_WALL) // Checking if there is a wall on the left
        {
            mouse_maze[p_mouse->pos_y][p_mouse->pos_x] |= WEST_WALL; // Add left wall
            if (p_mouse->pos_x != 0)
                mouse_maze[p_mouse->pos_y][p_mouse->pos_x - 1] |= EAST_WALL; // Add right wall to the previous column
        }

        // Single destination mode: no automatic destination expansion.

        mouse_maze[p_mouse->pos_y][p_mouse->pos_x] |= CELL_VISITED;
        p_mouse->cells_found += 1;
    }

    // Checking if Destination has been found
    if (p_mouse->destination_found == 0)
        if ((p_mouse->pos_y == Y_DEST) && (p_mouse->pos_x == X_DEST))
        {
            p_mouse->destination_found = 1;
            p_mouse->dest_cells_found = TOT_DEST_CELLS;
        }

    // Dynamic CELL_OUT marking is disabled for this fixed channel map.
}

void Solving(unsigned int mouse_maze[SIZE][SIZE], unsigned int discovered_cells[SIZE][SIZE])
{
    unsigned int r, c, r_check, c_check, cell_check;

    mouse_maze[0][0] |= CELL_USED; // First cell will always be used

    // Store order number of the cell destination
    cell_check = discovered_cells[Y_DEST][X_DEST];

    // Following discovered cells order
    while (cell_check != 1)
    {
        for (r = 0; r < SIZE; r++)
            for (c = 0; c < SIZE; c++)
                if (discovered_cells[r][c] == cell_check)
                {
                    mouse_maze[r][c] |= CELL_USED;
                    r_check = r;
                    c_check = c; // storing found position
                }

        // Checking around current "cell_check" which cell has the next LOWEST order value

        // NORTH value check
        if ((mouse_maze[r_check][c_check] & NORTH_WALL) != NORTH_WALL)
            if ((discovered_cells[r_check + 1][c_check] < cell_check) && (discovered_cells[r_check + 1][c_check] > 0))
                cell_check = discovered_cells[r_check + 1][c_check];

        // EAST value check
        if ((mouse_maze[r_check][c_check] & EAST_WALL) != EAST_WALL)
            if ((discovered_cells[r_check][c_check + 1] < cell_check) && (discovered_cells[r_check][c_check + 1] > 0))
                cell_check = discovered_cells[r_check][c_check + 1];

        // SOUTH value check
        if ((mouse_maze[r_check][c_check] & SOUTH_WALL) != SOUTH_WALL)
            if ((discovered_cells[r_check - 1][c_check] < cell_check) && (discovered_cells[r_check - 1][c_check] > 0))
                cell_check = discovered_cells[r_check - 1][c_check];

        // WEST value check
        if ((mouse_maze[r_check][c_check] & WEST_WALL) != WEST_WALL)
            if ((discovered_cells[r_check][c_check - 1] < cell_check) && (discovered_cells[r_check][c_check - 1] > 0))
                cell_check = discovered_cells[r_check][c_check - 1];
    }
}

void ToDest(unsigned int mouse_maze[SIZE][SIZE], struct Mouse_Settings *p_mouse)
{
    // Storing current cell mapping
    p_mouse->prev_cell_x = p_mouse->pos_x;
    p_mouse->prev_cell_y = p_mouse->pos_y;

    while ((mouse_maze[p_mouse->pos_y][p_mouse->pos_x] & DEST_CELL) != DEST_CELL)
    {
        // Moving towards the direction of the next cell SET AS USED
        // Also, making sure it does not go to previous cell

        if (
            ((mouse_maze[p_mouse->pos_y][p_mouse->pos_x] & NORTH_WALL) != NORTH_WALL) &&
            ((p_mouse->pos_y + 1) != p_mouse->prev_cell_y) &&
            ((mouse_maze[p_mouse->pos_y + 1][p_mouse->pos_x] & CELL_USED) == CELL_USED))
        {
            p_mouse->prev_cell_y = p_mouse->pos_y;
            p_mouse->pos_y += 1;
        }
        else if (
            ((mouse_maze[p_mouse->pos_y][p_mouse->pos_x] & EAST_WALL) != EAST_WALL) &&
            ((p_mouse->pos_x + 1) != p_mouse->prev_cell_x) &&
            ((mouse_maze[p_mouse->pos_y][p_mouse->pos_x + 1] & CELL_USED) == CELL_USED))
        {
            p_mouse->prev_cell_x = p_mouse->pos_x;
            p_mouse->pos_x += 1;
        }
        else if (
            ((mouse_maze[p_mouse->pos_y][p_mouse->pos_x] & SOUTH_WALL) != SOUTH_WALL) &&
            ((p_mouse->pos_y - 1) != p_mouse->prev_cell_y) &&
            ((mouse_maze[p_mouse->pos_y - 1][p_mouse->pos_x] & CELL_USED) == CELL_USED))
        {
            p_mouse->prev_cell_y = p_mouse->pos_y;
            p_mouse->pos_y -= 1;
        }
        else if (
            ((mouse_maze[p_mouse->pos_y][p_mouse->pos_x] & WEST_WALL) != WEST_WALL) &&
            ((p_mouse->pos_x - 1) != p_mouse->prev_cell_x) &&
            ((mouse_maze[p_mouse->pos_y][p_mouse->pos_x - 1] & CELL_USED) == CELL_USED))
        {
            p_mouse->prev_cell_x = p_mouse->pos_x;
            p_mouse->pos_x -= 1;
        }
    }
}

int main()
{
    unsigned int maze[HEIGHT][LENGTH];
    unsigned int mouse_maze[SIZE][SIZE];

    // Maze Generation and Simulation
    Generate_Maze(maze);
    Generate_Channel_Maze(maze);
    Print_Maze(maze);
    Print_ASCII_Walls(maze);

    // Preparing Initial Mouse Setup
    struct Mouse_Settings mouse;
    struct Mouse_Settings *p_mouse = &mouse;
    Mouse_Setup(p_mouse);
    Mouse_Maze_Setup(mouse_maze);

    unsigned int count = 2, discovered_cells[SIZE][SIZE];
    int r, c;
    // Setting the Temporary Maze
    for (r = 0; r < SIZE; r++)
        for (c = 0; c < SIZE; c++)
            discovered_cells[r][c] = 0;
    discovered_cells[0][0] = 1;

    /// print all paths
    char path_viz[SIZE][SIZE]; memset(path_viz, ' ', sizeof(path_viz)); int step=1; path_viz[0][0] = 'S';
    ////////////

    while ((p_mouse->cells_found != TOT_CELLS) || (p_mouse->pos_y != 0) || (p_mouse->pos_x != 0))
    {
        Mouse_Exploring(maze, mouse_maze, p_mouse, discovered_cells);
        Walls_Check(maze, p_mouse, mouse_maze);

        //print all paths
        // path[path_len][0] = p_mouse->pos_y; path[path_len][1] = p_mouse->pos_x; path_len++;


        // Editing the Maze table showing the movement of the mouse
        if (discovered_cells[p_mouse->pos_y][p_mouse->pos_x] == 0)
        {
            discovered_cells[p_mouse->pos_y][p_mouse->pos_x] = count;
            count++;
        }
        //path prints
        if(path_viz[p_mouse->pos_y][p_mouse->pos_x] == ' ') { path_viz[p_mouse->pos_y][p_mouse->pos_x] = (step%10)+'0'; step++; }
    }


    Solving(mouse_maze, discovered_cells);
    Print_Mouse_Maze(mouse_maze);
    Print_ASCII_Maze(maze, mouse_maze);
    // ToDest(mouse_maze, p_mouse);
    //printf("--- Full Mouse Path ---\n"); for(int i=0; i<path_len; i++) printf("(%d,%d) ", path[i][0], path[i][1]); printf("\n");
printf("--- ASCII Path Viz ---\n"); for(int r=SIZE-1; r>=0; r--) { for(int c=0; c<SIZE; c++) printf("%c ", path_viz[r][c]); printf("\n"); }

    // Visualizing Movement
    printf("--- Mouse Pattern ---\n");
    for (r = (SIZE - 1); r > -1; r--)
    {
        for (c = 0; c < SIZE; c++)
            printf("%d\t", discovered_cells[r][c]);
        printf("\n");
    }
    printf("\n\n");

    // Visualizing last 8 cells visited
    printf("--- Last Visited Cells ---\n");
    for (r = (SIZE - 1); r > -1; r--)
    {
        for (c = 0; c < SIZE; c++)
            printf("%d\t", p_mouse->prev_cells[r][c]);
        printf("\n");
    }
    printf("\n\n");

    printf("pos_y = %d\tpos_x = %d\tcells_found = %d\tm-dir = %d", p_mouse->pos_y, p_mouse->pos_x, p_mouse->cells_found, p_mouse->m_dir);

    return 0;
}
