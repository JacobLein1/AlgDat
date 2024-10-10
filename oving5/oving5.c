# include <stdio.h>
# include <stdlib.h>
# include <string.h>


// ------------------------------------ //
//              Linked List             //
// ------------------------------------ //

typedef struct node {
  int value;
  struct node *next;
} node;

node *insert_front(node *head, int value) {
  node *new_node = malloc(sizeof(node));
  if (new_node == NULL) {
    // Handle memory allocation failure
    fprintf(stderr, "Memory allocation failed\n");
    exit(EXIT_FAILURE);
  }
  new_node->value = value;
  new_node->next = head;
  return new_node;
}

node *search(node *head, int value) {
  node *current = head;
  while (current != NULL) {
    if (current->value == value) {
      return current;
    }
    current = current->next;
  }
  return NULL;
}

void free_list(node* head) {
    node* current = head;
    while (current != NULL) {
        node* temp = current;  // Keep track of the current node to free it
        current = current->next; // Move to the next node
        free(temp); // Free the current node
    }
}

// ------------------------------------ //
//                Grafs                 //
// ------------------------------------ //

typedef struct graph {
  int amt_node;
  int amt_edge;
  node **table;
} graph;

// Lag omvendt graf
graph* reverse_graph(graph* graph) {
  // Make a new graph
  struct graph* reversed_graph = malloc(sizeof(struct graph));
  reversed_graph->amt_node = graph->amt_node;
  reversed_graph->amt_edge = graph->amt_edge;
  reversed_graph->table = malloc(sizeof(node*) * graph->amt_node);

  int amt_node = graph->amt_node;
  node** reversed_table = reversed_graph->table;
  for (int i = 0; i < amt_node; i++)
  {
    reversed_table[i] = NULL;
  }

  for (int i = 0; i < amt_node; i++)
  {
    node* adj_list = graph->table[i];
    while (adj_list != NULL) {
      int adj_vertex = adj_list->value;
      reversed_table[adj_vertex] = insert_front(reversed_table[adj_vertex], i);
      adj_list = adj_list->next;
    }
  }

  return reversed_graph;
}

void free_graph(graph* g) {
  for (int i = 0; i < g->amt_node; i++) {
    free_list(g->table[i]);
  }
  free(g->table);
  free(g);
}

// ------------------------------------ //
//             File handler             //
// ------------------------------------ //

graph* read_file(char *filename) {
  // Open file
  FILE *file = fopen(filename, "r");
  if (file == NULL) {
    printf("Could not open file %s\n", filename);
    exit(1);
  }

  // Create graph
  graph *g = malloc(sizeof(graph));

  // Read metadata in first line
  int amt_node, amt_edge;
  fscanf(file, " %d %d", &amt_node, &amt_edge);
  printf("noder: %d, kanter: %d\n", amt_node, amt_edge);
  g->amt_node = amt_node;
  g->amt_edge = amt_edge;

  // Create table of linked lists for each node
  node** table = malloc(sizeof(node*) * amt_node);
  for (int i = 0; i < amt_node; i++)
  {
    table[i] = NULL;
  }
  g->table = table;


  for (int i = 0; i < amt_edge; i++)
  {
    // Read edge
    int from, to;
    fscanf(file, " %d %d", &from, &to);
    // printf("fra: %d, til: %d\n", from, to);

    // store edge
    table[from] = insert_front(table[from], to);
  }


  fclose(file);
  return g;
}

// ------------------------------------ //
//                 DFS                  //
// ------------------------------------ //
void dfs(node** table, int start, int* visited, node** order) {
  // Mark the current node as visited
  visited[start] = 1;
  // printf("Visited %d\n", start);

  // Add the current node to the order
  *order = insert_front(*order, start);

  // Recur for all the nodes adjacent to this vertex
  node* adj_list = table[start];
  while (adj_list != NULL) {
    int adj_vertex = adj_list->value;
    if (!visited[adj_vertex]) {
      dfs(table, adj_vertex, visited, order);
    }
    adj_list = adj_list->next;
  }
}

void print_SSK(graph* g) {
  node** order = NULL; // Head of linked list

  // Initialize visited array
  int* visited = malloc(sizeof(int) * g->amt_node);
  for (int i = 0; i < g->amt_node; i++) {
    visited[i] = 0;
  }

  // Perform DFS until all nodes are visited
  for (int i = 0; i < g->amt_node; i++) {
    if (visited[i] == 0) {
      // ikke besøkt i
      dfs(g->table, i, visited, &order);
    }
  }

  // Reverse the graph
  graph* reversed_g = reverse_graph(g);

  // Initialize visited array
  for (int i = 0; i < g->amt_node; i++) {
    visited[i] = 0;
  }

  // Perform DFS until all nodes are visited
  node* current = order;
  while (current != NULL) {
    if (visited[current->value] == 1) {
      current = current->next;
      continue;
    }

    node** SSK = NULL; // Head of linked list

    // Perform DFS from current node
    int current_node = current->value;
    dfs(reversed_g->table, current_node, visited, &SSK);

    // Print SSK
    node* current_ssk = SSK;
    while (current_ssk != NULL) {
      printf("%d ", current_ssk->value);
      current_ssk = current_ssk->next;
    }
    printf("\n");

    // Free memory
    free_list(SSK);

    current = current->next;
  }

  // Free memory
  free(visited);
  free_list(order);
  free_graph(reversed_g);
}

// ------------------------------------ //
//                Main                  //
// ------------------------------------ //

int main()
{
  graph* g = read_file("o5g6.txt");

  print_SSK(g);

  free_graph(g);

  return 0;
}
