#!/usr/bin/env python3
from box_place.inventory_check_algorithm import InventoryCheckAlgorithm
from box_comm.inventory_check_interface import start_algorithm


def main(args=None):
    start_algorithm(InventoryCheckAlgorithm, args=args)


if __name__ == '__main__':
    main()
