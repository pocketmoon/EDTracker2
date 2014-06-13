class iLowPass {
    IntList  buffer;
    int len;
    int output;

    iLowPass(int len) {
        this.len = len;
        buffer = new IntList();
        for(int i = 0; i < len; i++) {
            buffer.append(0);
        }
    }

    void input(int v) {
        buffer.append(v);
        buffer.remove(0);

        int  sum = 0;
        for(int i=0; i<buffer.size(); i++) {
                int fv = buffer.get(i);
                sum += fv;
        }
        output = sum / buffer.size();
    }
}
