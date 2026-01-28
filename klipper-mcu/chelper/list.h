/**
 * @file    list.h
 * @brief   Linked list utilities for Klipper MCU
 * 
 * Adapted from Klipper klippy/chelper/list.h for MCU use.
 * Provides doubly-linked list macros and inline functions.
 * 
 * Key adaptations:
 * - C99 compatible (no typeof extension)
 * - Uses explicit type parameters in macros
 */

#ifndef CHELPER_LIST_H
#define CHELPER_LIST_H

#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief List node structure for doubly-linked lists
 */
struct list_node {
    struct list_node *next;
    struct list_node *prev;
};

/**
 * @brief List head structure (sentinel node)
 */
struct list_head {
    struct list_node root;
};

/**
 * @brief Get the struct containing this list node
 * @param ptr    Pointer to the list_node member
 * @param type   Type of the containing struct
 * @param member Name of the list_node member in the struct
 */
#define container_of(ptr, type, member) \
    ((type *)((char *)(ptr) - offsetof(type, member)))

/**
 * @brief Get the struct containing this list node
 */
#define list_entry(ptr, type, member) \
    container_of(ptr, type, member)

/**
 * @brief Initialize a list head statically
 */
#define LIST_HEAD_INIT(name) { { &(name).root, &(name).root } }

/**
 * @brief Declare and initialize a list head
 */
#define LIST_HEAD(name) \
    struct list_head name = LIST_HEAD_INIT(name)

/**
 * @brief Initialize a list head at runtime
 */
static inline void
list_init(struct list_head *head)
{
    head->root.next = &head->root;
    head->root.prev = &head->root;
}

/**
 * @brief Check if list is empty
 */
static inline int
list_empty(const struct list_head *head)
{
    return head->root.next == &head->root;
}

/**
 * @brief Get first entry in list (or NULL if empty)
 * @param head   Pointer to list_head
 * @param type   Type of the containing struct
 * @param member Name of the list_node member
 */
#define list_first_entry(head, type, member) \
    (list_empty(head) ? NULL : list_entry((head)->root.next, type, member))

/**
 * @brief Get last entry in list (or NULL if empty)
 * @param head   Pointer to list_head
 * @param type   Type of the containing struct
 * @param member Name of the list_node member
 */
#define list_last_entry(head, type, member) \
    (list_empty(head) ? NULL : list_entry((head)->root.prev, type, member))

/**
 * @brief Get next entry in list
 * @param pos    Current entry pointer
 * @param type   Type of the containing struct
 * @param member Name of the list_node member
 */
#define list_next_entry(pos, type, member) \
    list_entry((pos)->member.next, type, member)

/**
 * @brief Get previous entry in list
 * @param pos    Current entry pointer
 * @param type   Type of the containing struct
 * @param member Name of the list_node member
 */
#define list_prev_entry(pos, type, member) \
    list_entry((pos)->member.prev, type, member)

/**
 * @brief Insert a new node between two known consecutive nodes
 */
static inline void
list_add_between(struct list_node *node, struct list_node *prev,
                 struct list_node *next)
{
    next->prev = node;
    node->next = next;
    node->prev = prev;
    prev->next = node;
}

/**
 * @brief Add node to the front of the list (after head)
 */
static inline void
list_add_head(struct list_node *node, struct list_head *head)
{
    list_add_between(node, &head->root, head->root.next);
}

/**
 * @brief Add node to the end of the list (before head)
 */
static inline void
list_add_tail(struct list_node *node, struct list_head *head)
{
    list_add_between(node, head->root.prev, &head->root);
}

/**
 * @brief Remove a node from its list
 */
static inline void
list_del(struct list_node *node)
{
    struct list_node *prev = node->prev;
    struct list_node *next = node->next;
    prev->next = next;
    next->prev = prev;
    /* Poison pointers to catch use-after-remove bugs */
    node->next = NULL;
    node->prev = NULL;
}

/**
 * @brief Iterate over list entries (C99 compatible)
 * @param pos    Loop variable (pointer to entry type)
 * @param head   Pointer to list_head
 * @param type   Type of the containing struct
 * @param member Name of the list_node member
 * 
 * Usage:
 *   struct my_struct *item;
 *   list_for_each_entry(item, &my_list, struct my_struct, node) {
 *       // use item
 *   }
 */
#define list_for_each_entry(pos, head, type, member) \
    for (pos = list_entry((head)->root.next, type, member); \
         &pos->member != &(head)->root; \
         pos = list_entry(pos->member.next, type, member))

/**
 * @brief Iterate over list entries (safe for removal, C99 compatible)
 * @param pos    Loop variable (pointer to entry type)
 * @param n      Temporary variable (same type as pos)
 * @param head   Pointer to list_head
 * @param type   Type of the containing struct
 * @param member Name of the list_node member
 * 
 * Usage:
 *   struct my_struct *item, *tmp;
 *   list_for_each_entry_safe(item, tmp, &my_list, struct my_struct, node) {
 *       list_del(&item->node);
 *       // free item
 *   }
 */
#define list_for_each_entry_safe(pos, n, head, type, member) \
    for (pos = list_entry((head)->root.next, type, member), \
         n = list_entry(pos->member.next, type, member); \
         &pos->member != &(head)->root; \
         pos = n, n = list_entry(n->member.next, type, member))

#ifdef __cplusplus
}
#endif

#endif /* CHELPER_LIST_H */
