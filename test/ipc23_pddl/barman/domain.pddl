(define (domain barman)
(:requirements
    :strips
    :typing
    :negative-preconditions
    :disjunctive-preconditions
    :equality
    :existential-preconditions
    :universal-preconditions
    :conditional-effects
    :action-costs
)
(:types
    hand - object
    level - object
    beverage - object
    dispenser - object
    container - object
    ingredient - beverage
    cocktail - beverage
    shot - container
    shaker - container
)
(:constants
    shaker1 - shaker
    left - hand
    right - hand
    shot1 - shot
    shot2 - shot
    shot3 - shot
    shot4 - shot
    ingredient1 - ingredient
    ingredient2 - ingredient
    ingredient3 - ingredient
    cocktail1 - cocktail
    cocktail2 - cocktail
    cocktail3 - cocktail
    dispenser1 - dispenser
    dispenser2 - dispenser
    dispenser3 - dispenser
    l0 - level
    l1 - level
    l2 - level
)
(:predicates
    (ontable ?x0 - container)
    (holding ?x0 - hand ?x1 - container)
    (handempty ?x0 - hand)
    (empty ?x0 - container)
    (contains ?x0 - container ?x1 - beverage)
    (clean ?x0 - container)
    (used ?x0 - container ?x1 - beverage)
    (dispenses ?x0 - dispenser ?x1 - ingredient)
    (shaker-empty-level ?x0 - shaker ?x1 - level)
    (shaker-level ?x0 - shaker ?x1 - level)
    (next ?x0 - level ?x1 - level)
    (unshaked ?x0 - shaker)
    (shaked ?x0 - shaker)
    (cocktail-part1 ?x0 - cocktail ?x1 - ingredient)
    (cocktail-part2 ?x0 - cocktail ?x1 - ingredient)
)
(:functions
    (total-cost)
)
(:action grasp
    :parameters (?h - hand ?c - container)
    :precondition (and (ontable ?c) (handempty ?h))
    :effect (and (not (ontable ?c)) (not (handempty ?h)) (holding ?h ?c) (increase (total-cost) 1))
)

(:action leave
    :parameters (?h - hand ?c - container)
    :precondition (and (holding ?h ?c))
    :effect (and (not (holding ?h ?c)) (handempty ?h) (ontable ?c) (increase (total-cost) 1))
)

(:action fill-shot
    :parameters (?s - shot ?i - ingredient ?h1 - hand ?h2 - hand ?d - dispenser)
    :precondition (and (not (= ?h1 ?h2)) (holding ?h1 ?s) (handempty ?h2) (dispenses ?d ?i) (empty ?s) (clean ?s))
    :effect (and (not (empty ?s)) (contains ?s ?i) (not (clean ?s)) (used ?s ?i) (increase (total-cost) 10))
)

(:action refill-shot
    :parameters (?s - shot ?i - ingredient ?h1 - hand ?h2 - hand ?d - dispenser)
    :precondition (and (not (= ?h1 ?h2)) (holding ?h1 ?s) (handempty ?h2) (dispenses ?d ?i) (empty ?s) (used ?s ?i))
    :effect (and (not (empty ?s)) (contains ?s ?i) (increase (total-cost) 10))
)

(:action empty-shot
    :parameters (?h - hand ?p - shot ?b - beverage)
    :precondition (and (holding ?h ?p) (contains ?p ?b) (not (= ?p shot2)) (not (= ?p shot1)))
    :effect (and (not (contains ?p ?b)) (empty ?p) (increase (total-cost) 1))
)

(:action clean-shot
    :parameters (?s - shot ?b - beverage ?h1 - hand ?h2 - hand)
    :precondition (and (not (= ?h1 ?h2)) (holding ?h1 ?s) (handempty ?h2) (empty ?s) (used ?s ?b))
    :effect (and (not (used ?s ?b)) (clean ?s) (increase (total-cost) 1))
)

(:action pour-shot-to-clean-shaker
    :parameters (?s - shot ?i - ingredient ?d - shaker ?h1 - hand ?l - level ?l1 - level)
    :precondition (and (holding ?h1 ?s) (contains ?s ?i) (empty ?d) (clean ?d) (shaker-level ?d ?l) (next ?l ?l1))
    :effect (and (not (contains ?s ?i)) (empty ?s) (contains ?d ?i) (not (empty ?d)) (not (clean ?d)) (unshaked ?d) (not (shaker-level ?d ?l)) (shaker-level ?d ?l1) (increase (total-cost) 1))
)

(:action pour-shot-to-used-shaker-2
    :parameters (?s - shot ?i - ingredient ?d - shaker ?h1 - hand ?l - level ?l1 - level)
    :precondition (and (holding ?h1 ?s) (contains ?s ?i) (unshaked ?d) (shaker-level ?d ?l) (next ?l ?l1))
    :effect (and (not (contains ?s ?i)) (contains ?d ?i) (empty ?s) (not (shaker-level ?d ?l)) (shaker-level ?d ?l1) (increase (total-cost) 1))
)

(:action empty-shaker
    :parameters (?h - hand ?s - shaker ?b - cocktail ?l - level ?l1 - level)
    :precondition (and (holding ?h ?s) (contains ?s ?b) (shaked ?s) (shaker-level ?s ?l) (shaker-empty-level ?s ?l1))
    :effect (and (not (shaked ?s)) (not (shaker-level ?s ?l)) (shaker-level ?s ?l1) (not (contains ?s ?b)) (empty ?s) (increase (total-cost) 1))
)

(:action clean-shaker
    :parameters (?h1 - hand ?h2 - hand ?s - shaker)
    :precondition (and (not (= ?h1 ?h2)) (holding ?h1 ?s) (handempty ?h2) (empty ?s))
    :effect (and (clean ?s) (increase (total-cost) 1))
)

(:action shake
    :parameters (?b - cocktail ?d1 - ingredient ?d2 - ingredient ?s - shaker ?h1 - hand ?h2 - hand)
    :precondition (and (not (= ?h1 ?h2)) (holding ?h1 ?s) (handempty ?h2) (contains ?s ?d1) (contains ?s ?d2) (cocktail-part1 ?b ?d1) (cocktail-part2 ?b ?d2) (unshaked ?s))
    :effect (and (not (unshaked ?s)) (not (contains ?s ?d1)) (not (contains ?s ?d2)) (shaked ?s) (contains ?s ?b) (increase (total-cost) 1))
)

(:action pour-shaker-to-shot
    :parameters (?b - beverage ?d - shot ?h - hand ?s - shaker ?l - level ?l1 - level)
    :precondition (and (holding ?h ?s) (shaked ?s) (empty ?d) (clean ?d) (contains ?s cocktail2) (shaker-level ?s ?l) (next ?l1 ?l) (= ?b cocktail2) (not (= ?d shot2)))
    :effect (and (not (clean ?d)) (not (empty ?d)) (contains ?d cocktail2) (shaker-level ?s ?l1) (not (shaker-level ?s ?l)) (increase (total-cost) 1))
)

(:action empty-shot-2
    :parameters (?h - hand ?p - shot ?b - beverage)
    :precondition (and (holding ?h ?p) (contains ?p ?b) (not (= ?b cocktail2)) (not (= ?b cocktail1)) (not (= ?b cocktail3)))
    :effect (and (not (contains ?p ?b)) (empty ?p) (increase (total-cost) 1))
)

(:action pour-shaker-to-shot-2
    :parameters (?b - beverage ?d - shot ?h - hand ?s - shaker ?l - level ?l1 - level)
    :precondition (and (holding ?h ?s) (shaked ?s) (empty ?d) (clean ?d) (contains ?s cocktail1) (shaker-level ?s ?l) (next ?l1 ?l) (= ?b cocktail1) (not (= ?d shot1)))
    :effect (and (not (clean ?d)) (not (empty ?d)) (contains ?d cocktail1) (shaker-level ?s ?l1) (not (shaker-level ?s ?l)) (increase (total-cost) 1))
)

(:action pour-shaker-to-shot-3
    :parameters (?b - beverage ?d - shot ?h - hand ?s - shaker ?l - level ?l1 - level)
    :precondition (and (holding ?h ?s) (shaked ?s) (empty ?d) (clean ?d) (contains ?s ?b) (shaker-level ?s ?l) (next ?l1 ?l) (not (= ?d shot2)) (not (= ?d shot1)))
    :effect (and (not (clean ?d)) (not (empty ?d)) (contains ?d ?b) (shaker-level ?s ?l1) (not (shaker-level ?s ?l)) (increase (total-cost) 1))
)

)
