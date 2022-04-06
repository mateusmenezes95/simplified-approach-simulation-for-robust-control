function [] = ask_to_continue(ask)
    if ask
        answer = yes_or_no ("Do you want to continue");
        if not(answer)
            error("Exiting...")
        end
    end
end
