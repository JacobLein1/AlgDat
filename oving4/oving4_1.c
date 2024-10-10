
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#define TABLE_SIZE 167
// Noden for den koblede listen for å håndtere kollisjoner


// Link til nedlasting av fil MÅ LEGGES TIL LOKALT FOR Å KJØRE


typedef struct Node {

char *key;

struct Node *next;
} Node;
// Hashtabell-struktur
typedef struct HashTable {

Node **table;
} HashTable;
// Hash-funksjon
unsigned int hash(const char *key) {

unsigned int hash = 0;

while (*key) {

hash = (hash * 7) + *key++;

}

return hash % TABLE_SIZE;
}
// Initialiserer hashtabellen
HashTable* create_table() {

// Dette er en peker til listen av pekere til noder

HashTable *new_table = malloc(sizeof(HashTable));

// Allokerer minne til listen av pekere

new_table->table = malloc(sizeof(Node*) * TABLE_SIZE);

// Det er ingen noder i tabellen ennå så alle pekerne settes til NULL

for (int i = 0; i < TABLE_SIZE; i++) {

new_table->table[i] = NULL;

}

return new_table;
}
// Setter inn en streng i hashtabellen. Returnerer true hvis det var en kollisjon
bool insert(HashTable *table, const char *key) {

bool has_collision = false;

// Finner indeksen til strengen i hashtabellen

unsigned int index = hash(key);

// Lager plass til en ny node

Node *new_node = malloc(sizeof(Node));

// Kopierer strengen inn i noden

new_node->key = strdup(key);

if (table->table[index] != NULL) {

// Hvis listen ikke er tom så er det en kollisjon her. Da printer vi en melding og singnaliserer dette med en bool.

has_collision = true;

printf("Collision between %s and %s at index %d\n", table->table[index]->key, key, index);

}

// Setter inn noden i starten av listen

new_node->next = table->table[index];

// Oppdaterer pekeren til å peke på den nye starten av listen

table->table[index] = new_node;

return has_collision;
}
// Søker etter en streng i hashtabellen
char* search(HashTable *table, const char *key) {

// Finner indeksen til strengen i hashtabellen

unsigned int index = hash(key);

// Starter fra starten av listen

Node *current = table->table[index];

// Går gjennom listen og sammenligner strengene

while (current != NULL) {

if (strcmp(current->key, key) == 0) {

// Returnerer hvis strengen er funnet

return current->key;

}

// Går videre til neste node

current = current->next;

}

// Returnerer NULL hvis strengen ikke er funnet

return NULL;
}
// Fjerner en streng fra hashtabellen

void delete(HashTable *table, const char *key) {

unsigned int index = hash(key);

Node *current = table->table[index];

Node *prev = NULL;

// Går gjennom listen for å lete etter strengen

while (current != NULL && strcmp(current->key, key) != 0) {

prev = current;

current = current->next;

}

if (current == NULL) {

// Strengen ble ikke funnet ved å gå gjennom listen

printf("Key not found: %s\n", key);

return;

}

if (prev == NULL) {

// Sletter den første noden i listen

table->table[index] = current->next;

} else {

// Sletter en node som ikke er den første i listen

prev->next = current->next;

}

// Frigjør minnet brukt av noden

free(current->key);

free(current);
}
// Frigjør minnet brukt av hashtabellen
void free_table(HashTable *table) {

if (table == NULL) {

return;

}

for (int i = 0; i < TABLE_SIZE; i++) {

Node *current = table->table[i];

while (current != NULL) {

Node *next = current->next;

free(current->key);

free(current);

current = next;

}

}

free(table->table);

free(table);
}
// Les alle navn i navn.txt og legg dem i en hashtabell
HashTable* read_names() {

int amt_names = 0;

int amt_collisions = 0;

// Oppretter en tom hashtabell

HashTable *table = create_table();

// Åpner filen for lesing

FILE *file = fopen("navn.txt", "r");

// Sjekker om filen ble åpnet

if (file == NULL) {

perror("Failed to open file");

exit(1);

}

// Leser navn fra filen og legger dem i hashtabellen. Vi antar at det er maks 100 tegn i hvert navn

char name[100];


// Leser en linje om gangen

while (fgets(name, 100, file) != NULL) {

name[strcspn(name, "\n")] = 0;

amt_names++;

// Setter inn navnet i hashtabellen

if (insert(table, name)) {

// Øker antall kollisjoner hvis det var en kollisjon

amt_collisions++;

}

}

// Lukker filen

fclose(file);

// Skriver ut antall kollisjoner og antall kollisjoner per innsettelse

printf("Amount of collisions: %d\n", amt_collisions);

printf("Collisions per insert: %f\n", (float)amt_collisions / amt_names);

// Returnerer hashtabellen


return table;
}
// Testprogram
int main() {
    // ----------------------------------- //
    //                                     //
    //                                     //
    //          Lese inn alle navn         //
    //                                     //
    //                                     //
    // ----------------------------------- //

    HashTable *table = read_names();

    // ----------------------------------- //
    //                                     //
    //                                     //
    //        Søke etter våre navn         //
    //                                     //
    //                                     //
    // ----------------------------------- //

    char *res = NULL;

    res = search(table, "Jacob Lein");

    if (res != NULL) {

        printf("Found: %s\n", res);

    } else {

        printf("Not found: Jacob Lein\n");

    }

    res = search(table, "Erik August Christensen Hoff");

    if (res != NULL) {

        printf("Found: %s\n", res);

    } else {

        printf("Erik August Christensen Hoff\n");

    }

    // ----------------------------------- //
    //                                     //
    //                                     //
    //            Sjekk Lastfaktor         //
    //                                     //
    //                                     //
    // ----------------------------------- //

    int amt_nodes = 0;

    for (int i = 0; i < TABLE_SIZE; i++) {

        Node *current = table->table[i];

        while (current != NULL) {

            amt_nodes++;

            current = current->next;

        }

    }

    printf("Load factor: %f\n", (float)amt_nodes / TABLE_SIZE);


    free_table(table);

    return 0;
}