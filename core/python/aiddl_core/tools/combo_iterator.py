

class ComboIterator:
    def __init__(self, choices):
        self.slot_sizes = []
        self.slots = []
        self.carry = False
        self.done = False

        self.choices = choices

        for i in range(len(choices)):
            self.slot_sizes.append(len(self.choices[i]))
            self.slots.append(0)
        if len(self.choices) == 0:
            self.done = True
            return
        for choice in choices:
            if len(choice) == 0:
                self.done = True
                return

    def get_num_combos(self):
        n = 1
        for k in self.slot_sizes:
            n *= k
        return n

    def has_next(self):
        return not self.done

    def __next__(self):
        if self.done:
            raise StopIteration
        combo = []
        for i in range(0, len(self.slots)):
            combo.append(self.choices[i][self.slots[i]])
        self.iterate()
        return combo

    def iterate(self):
        if self.done:
            self.done = False
            for i in range(len(self.slots)):
                self.slots[i] = 0
        self.carry = False
        self.slots[0] += 1
        for i in range(len(self.slots)):
            if self.carry:
                self.slots[i] += 1
                self.carry = False
            if self.slots[i] == self.slot_sizes[i]:
                self.slots[i] = 0
                self.carry = True
        if self.carry:
            self.done = True

    def __iter__(self):
        return self


__c = ComboIterator([[1, 2],
                     [3, 4]])
assert(list(__c) == [[1, 3], [2, 3], [1, 4], [2, 4]])
