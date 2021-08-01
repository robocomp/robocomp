//
// Created by juancarlos on 17/12/20.
//

#ifndef PYTHON_WRAPPER_SILENT_OUTPUT_H
#define PYTHON_WRAPPER_SILENT_OUTPUT_H


class scoped_ostream_discard
{
protected:
    std::streambuf *old;
    std::ostream &costream;

public:
    explicit scoped_ostream_discard(
            std::ostream &costream = std::cout)
            : costream(costream) {
        old = costream.rdbuf(nullptr);
    }

    ~scoped_ostream_discard() {
        costream.rdbuf(old);
    }

    scoped_ostream_discard(const scoped_ostream_discard &) = delete;

    scoped_ostream_discard(scoped_ostream_discard &&other) = default;

    scoped_ostream_discard &operator=(const scoped_ostream_discard &) = delete;

    scoped_ostream_discard &operator=(scoped_ostream_discard &&) = delete;
};


#endif //PYTHON_WRAPPER_SILENT_OUTPUT_H
